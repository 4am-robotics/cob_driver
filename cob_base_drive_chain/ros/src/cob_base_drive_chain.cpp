/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

//##################
//#### includes ####

// standard includes
//--
#include <sstream>

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <pr2_controllers_msgs/JointControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>

//cob includes:
#include <cob_srvs/Trigger.h>

// external includes

#include <cob_utilities/IniFile.h>
#include <cob_utilities/MathSup.h>



//internal includes:
#include <cob_base_drive_chain/NeoCtrlPltfMpo500.h>

//####################
//#### node class ####
/**
* This node provides control of the care-o-bot platform drives to the ROS-"network". For this purpose it offers several services and publishes data on different topics.
*/
class NodeClass
{
	public:
	// create a handle for this node, initialize node
	ros::NodeHandle n;
	ros::Publisher topicPub_JointState;
	ros::Publisher topicPub_Diagnostic;
	ros::Subscriber topicSub_JointStateCmd;

	// service servers
	ros::ServiceServer srvServer_Init;
	ros::ServiceServer srvServer_Recover;
	ros::ServiceServer srvServer_Shutdown;

	// global variables
	// generate can-node handle
	NeoCtrlPltfMpo500 *m_CanCtrlPltf;
	bool m_bisInitialized;
	//config parameters
	bool autoInit;
	bool m_bPubEffort;
	bool m_bReadoutElmo;

	// Constructor
	NodeClass();

	// Destructor
	~NodeClass();

	void sendVelCan();
	void topicCallback_JointStateCmd(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
	bool can_init();
	bool srv_can_init(cob_srvs::Trigger::Request& req, cob_srvs::Trigger::Response& res );
	bool recover();
	bool srv_recover(cob_srvs::Trigger::Request& req, cob_srvs::Trigger::Response& res );
	bool shutdown();
	bool srv_shutdown(cob_srvs::Trigger::Request& req, cob_srvs::Trigger::Response& res );
	bool publish_JointStates();
	bool initDrives();
	ros::Time last, now;

	private:
	void setBaseConfig();
	std::vector<std::string> joint_names;
	std::vector<double> max_drive_rates;
	std::vector<int> control_type;
	double *sendVel;
	double *sendPos;
	int* canIDs;
	int m_iNumMotors;
	bool bIsError;
	ros::Duration timeOut, auto_recover_interval;
	ros::Time last_cmd_time, last_recover_try, next_cmd_time;
	bool checkJointNames;
	trajectory_msgs::JointTrajectory traj_;
	int traj_point_;
};

//##################################
//#### function implementations ####


// Constructor
NodeClass::NodeClass() : auto_recover_interval(1.)
{
	// initialization of variables
	m_bisInitialized = false;

	//bIsError = true;
	/// Parameters are set within the launch file
	// Read number of drives from iniFile and pass IniDirectory to CobPlatfCtrl.

	traj_point_ = 0;

	n.param<bool>("PublishEffort", m_bPubEffort, true);
	n.param<bool>("AutoInit",autoInit, true);
	n.param<bool>("CheckJointNames",checkJointNames, false);
	ROS_INFO("autoinitializing base_drive_chain");
	//stop all Drives if no new cmd_vel have been received within $errorStopTime seconds.
	last_cmd_time = ros::Time::now();
	double errorStopTime;
	n.param<double>("TimeOut", errorStopTime, 2);
	ros::Duration d(errorStopTime);
	timeOut = d;
	// get max Joint-Velocities (in rad/s) for Steer- and Drive-Joint
	n.getParam("numberOfMotors", m_iNumMotors);
	sendVel = new double[m_iNumMotors];
	sendPos = new double[m_iNumMotors];
	canIDs = new int[m_iNumMotors];
	for(int i=0; i<m_iNumMotors; i++)
	{
		sendVel[i] = 0;
		sendPos[i] = 0;
	}
	joint_names.resize(m_iNumMotors);
	max_drive_rates.resize(m_iNumMotors);
	control_type.resize(m_iNumMotors);
	for(int i=0; i<m_iNumMotors; i++)
	{
		std::ostringstream stringstream;
		stringstream<<"drive"<<i<<"/";
		std::string pathName = stringstream.str();
		n.getParam(pathName + "joint_name", joint_names[i]);
		n.getParam(pathName + "CANId", canIDs[i]);
		n.getParam(pathName + "control_type", control_type[i]);

		//calulate max velocity:
		double velMaxEncIncrS, gearRatio, beltRatio, encIncrPerRevMot;
		n.getParam(pathName + "GearRatio", gearRatio);
		n.getParam(pathName + "BeltRatio", beltRatio);
		n.getParam(pathName + "EncIncrPerRevMot", encIncrPerRevMot);
		n.getParam(pathName + "VelMaxEncIncrS", velMaxEncIncrS);
		double m_dRadToIncr = (encIncrPerRevMot * gearRatio * beltRatio) / (2. * 3.14159265);
		max_drive_rates[i] = velMaxEncIncrS / m_dRadToIncr;
		
		ROS_DEBUG("motor nr: %i can id: %i, max_rad_per_s: %f",i, canIDs[i], max_drive_rates[i]);
	}
	m_CanCtrlPltf = new NeoCtrlPltfMpo500();
	setBaseConfig(); //configures m_CanCtrlPltf
	
	// implementation of topics
	// published topics
	topicPub_JointState = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	
	
	topicPub_Diagnostic = n.advertise<diagnostic_msgs::DiagnosticStatus>("diagnostic", 1);
	// subscribed topics
	topicSub_JointStateCmd = n.subscribe("cmd_joint_traj", 1, &NodeClass::topicCallback_JointStateCmd, this);
	if(!autoInit)
	{
		srvServer_Init = n.advertiseService("init", &NodeClass::srv_can_init, this);
		srvServer_Recover = n.advertiseService("recover", &NodeClass::srv_recover, this);
		srvServer_Shutdown = n.advertiseService("shutdown", &NodeClass::srv_shutdown, this);
	}
}

void NodeClass::setBaseConfig()
{
	////////////////////////
	// Parameters:

	//can device:
	int iTypeCan;	
	int iBaudrateVal;
	std::string* sCanDevice = 0;
	//loop rate
	int rate;
	//drives
	std::vector<DriveParam> DriveParamDriveMotor;
	std::vector<NeoCtrlPltfMpo500::GearMotorParamType> m_GearMotDrive;
	bool bHomeAllAtOnce;
	std::vector<int> control_type;
	std::vector<int> m_viMotorID;

	////////////////////////
	// get parameters:

	if( n.hasParam("devicePath") ){
			sCanDevice = new(std::string);
			n.getParam("devicePath", *sCanDevice);
	}
	// read Configuration of the Can-Network (CanCtrl.ini)
	n.getParam("can", iTypeCan);
	n.getParam("BaudrateVal", iBaudrateVal);
	n.getParam("cycleRate", rate);
	n.param<bool>("HomeAllAtOnce",bHomeAllAtOnce,false);
	//can settings:
	control_type.resize(m_iNumMotors);
	m_viMotorID.resize(m_iNumMotors);
	DriveParamDriveMotor.resize(m_iNumMotors);
	m_GearMotDrive.resize(m_iNumMotors);
	// "Drive Motor Type1" drive parameters
	for(int i=0; i<m_iNumMotors; i++)
	{

		std::ostringstream stringstream;
		stringstream<<"drive"<<i<<"/";
		std::string pathName = stringstream.str();
		n.getParam(pathName + "control_type", control_type[i]);
		n.getParam(pathName + "EncIncrPerRevMot", m_GearMotDrive[i].iEncIncrPerRevMot);
		n.getParam(pathName + "VelMeasFrqHz", m_GearMotDrive[i].dVelMeasFrqHz);
		n.getParam(pathName + "BeltRatio", m_GearMotDrive[i].dBeltRatio);
		n.getParam(pathName + "GearRatio", m_GearMotDrive[i].dGearRatio);
		n.getParam(pathName + "GearEfficiency",m_GearMotDrive[i].dGearEfficiency);
		n.getParam(pathName + "Sign", m_GearMotDrive[i].iSign);
		n.getParam(pathName + "Homing", m_GearMotDrive[i].bHoming);
		n.param<int>(pathName + "HomeCoupleID", m_GearMotDrive[i].iHomeCoupleID, -1);
		n.param<double>(pathName + "HomeCoupleVelRadS", m_GearMotDrive[i].iHomeCoupleVel, 0.2);
		n.getParam(pathName + "HomePos", m_GearMotDrive[i].dHomePos);
		n.getParam(pathName + "HomeVelRadS", m_GearMotDrive[i].dHomeVel);
		n.getParam(pathName + "HomeEvent", m_GearMotDrive[i].iHomeEvent);
		n.getParam(pathName + "HomeDigIn", m_GearMotDrive[i].iHomeDigIn);
		n.getParam(pathName + "HomeTimeOut", m_GearMotDrive[i].iHomeTimeOut);
		n.getParam(pathName + "CurrentToTorque", m_GearMotDrive[i].dCurrentToTorque);
		n.getParam(pathName + "CurrentContLimit", m_GearMotDrive[i].dCurrentContLimit);
		n.getParam(pathName + "VelMaxEncIncrS", m_GearMotDrive[i].dVelMaxEncIncrS);
		n.getParam(pathName + "VelPModeEncIncrS", m_GearMotDrive[i].dVelPModeEncIncrS);
		n.getParam(pathName + "AccIncrS", m_GearMotDrive[i].dAccIncrS2);
		n.getParam(pathName + "DecIncrS", m_GearMotDrive[i].dDecIncrS2);
		n.getParam(pathName + "CANId", m_GearMotDrive[i].iCANId);
		m_viMotorID[i] = m_GearMotDrive[i].iCANId;
		double 	m_dRadToIncr = 	(m_GearMotDrive[i].iEncIncrPerRevMot * m_GearMotDrive[i].dGearRatio * m_GearMotDrive[i].dBeltRatio) 
					/ (2. * 3.14159265);
		double homeVelIncrS = m_GearMotDrive[i].dHomeVel * m_dRadToIncr / m_GearMotDrive[i].dVelMeasFrqHz;


		DriveParamDriveMotor[i].set(	i,
						m_GearMotDrive[i].iEncIncrPerRevMot,
						m_GearMotDrive[i].dVelMeasFrqHz,
						m_GearMotDrive[i].dBeltRatio, m_GearMotDrive[i].dGearRatio,
						m_GearMotDrive[i].iSign,
						m_GearMotDrive[i].bHoming, m_GearMotDrive[i].dHomePos,
						homeVelIncrS, m_GearMotDrive[i].iHomeEvent,
						m_GearMotDrive[i].iHomeDigIn, m_GearMotDrive[i].iHomeTimeOut,
						m_GearMotDrive[i].dVelMaxEncIncrS, m_GearMotDrive[i].dVelPModeEncIncrS,
						m_GearMotDrive[i].dAccIncrS2, m_GearMotDrive[i].dDecIncrS2,
						DriveParam::ENCODER_INCREMENTAL,
						m_GearMotDrive[i].iCANId,
						false, true );



	}

 	m_CanCtrlPltf->readConfiguration(		iTypeCan, iBaudrateVal,	sCanDevice, rate,
							DriveParamDriveMotor, m_iNumMotors, m_GearMotDrive,
							bHomeAllAtOnce, control_type, m_viMotorID
	);

	if(sCanDevice) delete sCanDevice;
}

bool NodeClass::initDrives()
{
	ROS_INFO("Initializing Base Drive Chain");

	// debug log
	ROS_INFO("Initializing CanCtrlItf");
	bool bTemp1;
	bTemp1 =  m_CanCtrlPltf->initPltf();
	usleep(1000);
	bIsError = false;
	// debug log
	ROS_INFO("Initializing done");


	return bTemp1;
}

// Destructor
NodeClass::~NodeClass() 
{
	m_CanCtrlPltf->shutdownPltf();
	delete[] sendVel;
	delete[] sendPos;
	delete[] canIDs;
}

void  NodeClass::sendVelCan()
{
	if(m_bisInitialized == true && !bIsError)
	{
		if(traj_point_ < traj_.points.size()) //set next command
		{
			if(ros::Time::now() >= next_cmd_time)
			{

				last_cmd_time = ros::Time::now();
				if(ros::Duration(0.) == traj_.points[traj_point_].time_from_start) 
				{
					next_cmd_time = ros::Time::now() + ros::Duration(600.); //some time far away in the future
				}
				else
				{
					next_cmd_time = traj_.header.stamp + traj_.points[traj_point_].time_from_start;
				}
				if(checkJointNames)
				{
					for(int i = 0; i < m_iNumMotors; i++)
					{	
						int id = -1;
						for(int j = 0; j < traj_.joint_names.size(); j++)
						{
							if(traj_.joint_names[j] == joint_names[i])
							{
								id = j;
								break;
							}
						}
						if(id == -1)
						{
							for(int j = 0; j<m_iNumMotors; j++) sendVel[j] = 0;
							ROS_ERROR("cob_base_drive_chain: unknown joint names in trajectory cmd message");
							return;
						}
						if(traj_.points[traj_point_].velocities.size() > id) sendVel[i] = traj_.points[traj_point_].velocities[id];
						if(traj_.points[traj_point_].positions.size() > id) sendPos[i] = traj_.points[traj_point_].positions[id];
						if (sendVel[i] > max_drive_rates[id]) 		//TODO 1: throw error
												//TODO 2: are there better methods of handling these cases?
						{
							sendVel[i] = max_drive_rates[id];
						}
						if (sendVel[i] < -max_drive_rates[id])
						{
							sendVel[i] = -max_drive_rates[id];
						}
					}
				}
				else //assume fixed order
				{
					for(int i = 0; i < m_iNumMotors; i++)
					{	
						if(traj_.points[traj_point_].velocities.size() > i) sendVel[i] = traj_.points[traj_point_].velocities[i];
						if(traj_.points[traj_point_].positions.size() > i) sendPos[i] = traj_.points[traj_point_].positions[i];
						if (sendVel[i] > max_drive_rates[i])
						{
							sendVel[i] = max_drive_rates[i];
						}
						if (sendVel[i] < -max_drive_rates[i])
						{
							sendVel[i] = -max_drive_rates[i];
						}
					}

				}
				traj_point_++;
			}
		}
		if(timeOut > ros::Time::now() - last_cmd_time  ) //send velocity/position
		{
			for(int i=0; i<m_iNumMotors; i++){
				switch(control_type[i])
				{
					case 2: //velocity control:
						ROS_DEBUG("canid: %i set velocity %f in velocity mode",canIDs[i], sendVel[i]);
						m_CanCtrlPltf->setVelGearRadS(canIDs[i], sendVel[i]);
					break;
					case 1: //position control:
						ROS_DEBUG("canid: %i set position %f in pos mode",canIDs[i], sendPos[i]);
						m_CanCtrlPltf->setPosGearRad(canIDs[i], sendPos[i], sendVel[i]);
					break;
					case 0: //torque control: not supported yet.
					break;					

				}
			}
		}
		else //TODO: get this working again!
		{
			for(int i=0; i<m_iNumMotors; i++){
				switch(control_type[i])
				{
					case 2: //velocity control:
						m_CanCtrlPltf->setVelGearRadS(canIDs[i], 0.0);
						ROS_DEBUG("last velocity cmd timed out: set velocity to 0");
					case 1: //pose control:
						//TODO: what's the best thing to do? do nothing?
					break;
					case 0: //torque control: not supported yet.
					break;					

				}
			}
		}
	} 
	else 
	{
		ROS_DEBUG("error: can not initialized");
	}
}


// topic callback functions 
// function will be called when a new message arrives on a topic
void NodeClass::topicCallback_JointStateCmd(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
	ROS_DEBUG("Topic Callback joint_command");
	// only process cmds when system is initialized
	if(m_bisInitialized == true)
	{
		ROS_DEBUG("Topic Callback joint_command - Sending Commands to drives (initialized), nr of motors: %i", m_iNumMotors);
		traj_ = *msg;
		next_cmd_time = ros::Time::now();
		traj_point_ = 0;
	}
}


// Init Can-Configuration
bool NodeClass::srv_can_init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response& res )
{
	if(m_bisInitialized)
	{
		res.success.data = true;
		res.error_message.data = "platform already initialized";
		return true;
	} 
	can_init();
	res.success.data = m_bisInitialized;
	if(!m_bisInitialized) 
	{
		res.error_message.data = "initialization of base failed";
	}
}

bool NodeClass::can_init()
{
	ROS_DEBUG("Service Callback init");
	if(m_bisInitialized == false)
	{
		m_bisInitialized = initDrives();
		last = ros::Time::now();
		usleep(100000); //wait some time to ensure (now-last).toSec() isn't too small.
		//ROS_INFO("...initializing can-nodes...");

		if(m_bisInitialized)
		{
			bIsError = false;
   			ROS_INFO("base initialized");
		}
		else
		{
		  	ROS_ERROR("Initializing base failed");
		}
	}
	else
	{
		ROS_WARN("...base already initialized...");
	}
	return true;
}



// reset Can-Configuration
bool NodeClass::srv_recover(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response& res)
{
	if(!m_bisInitialized)
	{
		res.error_message.data = "failed to recover: base not initialized";
		res.success.data = false;
		return false;
	}
	res.success.data = recover();
	if(!res.success.data)
	{
		res.error_message.data = "base failed to recover";
	}
	return res.success.data;
}

bool NodeClass::recover()
{
	if(m_bisInitialized)
	{	
		usleep(100000);
		ROS_DEBUG("Service callback reset");
		bool reset = m_CanCtrlPltf->resetPltf();
		if (reset) {
			ROS_INFO("base resetted");
			bIsError = false;
		} else {
			ROS_DEBUG("Resetting base failed");
		}
		return reset;
	}
	else
	{
		ROS_WARN("...base not initialized...");
	}

	return false;
}

// shutdown Drivers and Can-Node
bool NodeClass::srv_shutdown(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response& res)
{
	res.success.data = shutdown();
	if(!res.success.data)
	{
		res.error_message.data = "base failed to shutdown";
	}
	return res.success.data;
}

bool NodeClass::shutdown()
{
	ROS_DEBUG("Service callback shutdown");
	bool success = m_CanCtrlPltf->shutdownPltf();
	if (success)
	{
		ROS_INFO("Drives shut down");
		return true;
	}
	else
		ROS_INFO("Shutdown of Drives FAILED");

	return false;
}

//publish JointStates cyclical instead of service callback
bool NodeClass::publish_JointStates()
{
	ROS_DEBUG("Service Callback GetJointState");
	// init local variables
	int j, k;
	std::vector<double> vdAngGearRad, vdVelGearRad, vdEffortGearNM;

	// set default values
	vdAngGearRad.resize(m_iNumMotors, 0);
	vdVelGearRad.resize(m_iNumMotors, 0);
	vdEffortGearNM.resize(m_iNumMotors, 0);

	// create temporary (local) JointState/Diagnostics Data-Container
	sensor_msgs::JointState jointstate;
	diagnostic_msgs::DiagnosticStatus diagnostics;
	
	// get time stamp for header
	jointstate.header.stamp = ros::Time::now();

	// assign right size to JointState
	//jointstate.name.resize(m_iNumMotors);
	jointstate.position.assign(m_iNumMotors, 0.0);
	jointstate.velocity.assign(m_iNumMotors, 0.0);
	jointstate.effort.assign(m_iNumMotors, 0.0);

	if(m_bisInitialized == false)
	{
		// as long as system is not initialized
		bIsError = false;
		// set data to jointstate			
		for(int i = 0; i<m_iNumMotors; i++)
		{
			jointstate.position[i] = 0.0;
			jointstate.velocity[i] = 0.0;
			jointstate.effort[i] = 0.0;
			jointstate.name.push_back(joint_names[i]);
		}
	}
	else
	{
		// as soon as drive chain is initialized
		// read Can-Buffer	
		if(m_iNumMotors > 0)
		{ 
			ROS_DEBUG("sending sync signal && evaluating can buffer");
			m_CanCtrlPltf->sendSynch();
			usleep(5000);
			m_CanCtrlPltf->evalCanBuffer();
		}
		j = 0;
		k = 0;
		for(int i = 0; i<m_iNumMotors; i++)
		{

			m_CanCtrlPltf->requestMotPosVel(canIDs[i]);
			m_CanCtrlPltf->getGearPosVelRadS(canIDs[i],  &vdAngGearRad[i], &vdVelGearRad[i]); 
			
			//Get motor torque
			if(m_bPubEffort) {
				m_CanCtrlPltf->getMotorTorque(canIDs[i], &vdEffortGearNM[i]); //(int iCanIdent, double* pdTorqueNm)
			}
		}
		// set data to jointstate
		for(int i = 0; i<m_iNumMotors; i++)
		{
			jointstate.position[i] = vdAngGearRad[i];
			jointstate.velocity[i] = vdVelGearRad[i];
			jointstate.effort[i] = vdEffortGearNM[i];
			jointstate.name.push_back(joint_names[i]);
		}
		
	}


	// publish jointstate message
	topicPub_JointState.publish(jointstate);
	
	ROS_DEBUG("published new drive-chain configuration (JointState message)");
	

	if(m_bisInitialized)
	{
		// read Can only after initialization
		// handle errors:
		bool old_errorState = bIsError;
		bIsError = m_CanCtrlPltf->isPltfError();
		if(bIsError)
		{
			ROS_DEBUG("platform has an error");
		}
		if(old_errorState == false && bIsError == true)
		{ 
			// a new error has occured..
			last_recover_try = ros::Time::now();
			m_CanCtrlPltf->stopPltf();
		}
	}

	// set data to diagnostics
	if(bIsError)
	{
		std::ostringstream error_msg;
		error_msg<<"platform error: can communication error\n";
		for(int i = 0; i<m_iNumMotors; i++)
		{
			int status, currentMeasPromille, tempCel;
			m_CanCtrlPltf->getStatus(canIDs[i], &status, &currentMeasPromille, &tempCel);
			if(status != 0)
			{
				// motor i has an failure
				if ( MathSup::isBitSet ( status, 2 ) ){
					error_msg << canIDs[i] << " feedback loss\n";
				}
				if ( MathSup::isBitSet ( status, 3 ) ){
					error_msg << canIDs[i] << " peak current exceeded\n";
				}
				if ( MathSup::isBitSet ( status, 4 ) ){
					error_msg << canIDs[i] <<" inhibit\n";
				}
				if ( MathSup::isBitSet ( status, 6 ) ){
					error_msg << canIDs[i] <<" Hall sensor error\n";
				}
				if ( MathSup::isBitSet ( status, 7 ) ){
					error_msg << canIDs[i] <<" speed track error\n";
				}
				if ( MathSup::isBitSet ( status, 8 ) ){
					error_msg << canIDs[i] <<" position track error\n";
				}
				if ( MathSup::isBitSet ( status, 9 ) ){
					error_msg << canIDs[i] <<" inconsistent database\n";
				}
				if ( MathSup::isBitSet ( status, 11 ) ){
					error_msg << canIDs[i] <<" heartbeat failure\n";
				}
				if ( MathSup::isBitSet ( status, 12 ) ){
					error_msg << canIDs[i] << " servo drive fault\n";
				}
				if ( ( status & 0x0E000 ) == 0x2000 ){
					error_msg << canIDs[i] <<" under voltage\n";
				}
				if ( ( status & 0x0E000 ) == 0x4000 ){
					error_msg << canIDs[i] <<" over voltage\n";
				}
				if ( ( status & 0x0E000 ) == 0xA000 ){
					error_msg << canIDs[i] <<" short circuit\n";
				}
				if ( ( status & 0x0E000 ) == 0xC000 ){
					error_msg << canIDs[i] <<" over temp\n";
				}
				if ( MathSup::isBitSet ( status, 16 ) ){
					error_msg << canIDs[i] <<" electrical zero not found\n";
				}
				if ( MathSup::isBitSet ( status, 17 ) ){
					error_msg << canIDs[i] <<" speed limit exceeded\n";
				}
				if ( MathSup::isBitSet ( status, 21 ) ){
					error_msg << canIDs[i] <<" motor stuck\n";
				}
				if ( MathSup::isBitSet ( status, 22 ) ){
					error_msg << canIDs[i] <<" position limit excceded\n";
				}
			}
		}	
		diagnostics.level = 2;
		diagnostics.name = "drive-chain can node";
		diagnostics.message = error_msg.str();
		if(autoInit)
		{
			if(ros::Time::now() - last_recover_try > auto_recover_interval)
			{
				last_recover_try = ros::Time::now();
				recover();
			}
		}
	}
	else
	{
		if (m_bisInitialized)
		{
			diagnostics.level = 0;
			diagnostics.name = "drive-chain can node";
			diagnostics.message = "drives operating normal";
		}
		else
		{
			diagnostics.level = 1;
			diagnostics.name = "drive-chain can node";
			diagnostics.message = "drives are initializing";
		}
	}

	// publish diagnostic message
	topicPub_Diagnostic.publish(diagnostics);
	ROS_DEBUG("published new drive-chain configuration (JointState message)");

	return true;
}

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "base_drive_chain");

	NodeClass nodeClass;

	// specify looprate of control-cycle
	int rate;
	nodeClass.n.getParam("cycleRate", rate);
 	ROS_INFO("set can querry rate to %i hz", rate);
	ros::Rate loop_rate(rate); // Hz
	nodeClass.last = ros::Time::now();
	nodeClass.now = ros::Time::now();
	if(nodeClass.autoInit)
	{
		nodeClass.can_init();
	}

	while(nodeClass.n.ok())
	{

		nodeClass.publish_JointStates();
		nodeClass.sendVelCan();
		loop_rate.sleep();
		ros::spinOnce();
		nodeClass.now = ros::Time::now();
		if(nodeClass.m_bisInitialized) nodeClass.m_CanCtrlPltf->timeStep( (nodeClass.now - nodeClass.last).toSec() );
		nodeClass.last = nodeClass.now;
	}

	return 0;
}


