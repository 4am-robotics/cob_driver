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

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <pr2_controllers_msgs/JointControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>

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

		// global variables
		// generate can-node handle
		NeoCtrlPltfMpo500 *m_CanCtrlPltf;
		bool m_bisInitialized;

		struct ParamType
		{ 
			double dMaxDriveRateRadpS;

			std::vector<double> vdWheelNtrlPosRad;
		};
		ParamType m_Param;
		
		bool m_bPubEffort;
		bool m_bReadoutElmo;

		// Constructor
		NodeClass();

		// Destructor
		~NodeClass() ;

		void sendVelCan();
		void topicCallback_JointStateCmd(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
		bool can_init();
		bool recover();
		bool shutdown();
		bool publish_JointStates();
		bool initDrives();

		private:
		std::vector<std::string> joint_names;
		double *sendVel;
		int m_iNumMotors;
		int reset_retries;
		bool bIsError;
};

//##################################
//#### function implementations ####


// Constructor
NodeClass::NodeClass()
{
	reset_retries = 0;
	// initialization of variables
	m_bisInitialized = false;

	bIsError = true;
	/// Parameters are set within the launch file
	// Read number of drives from iniFile and pass IniDirectory to CobPlatfCtrl.

	n.param<bool>("PublishEffort", m_bPubEffort, false);
	if(m_bPubEffort) ROS_INFO("You have choosen to publish effort of motors, that charges capacity of CAN");

	// get max Joint-Velocities (in rad/s) for Steer- and Drive-Joint
	n.getParam("numberOfMotors", m_iNumMotors);
	sendVel = new double[m_iNumMotors];
	for(int i=0; i<m_iNumMotors; i++) sendVel[i] = 0;
	joint_names.resize(m_iNumMotors);
	for(int i=0; i<m_iNumMotors; i++)
	{
		std::ostringstream stringstream;
		stringstream<<"drive"<<i<<"/";
		std::string pathName = stringstream.str();
		n.getParam(pathName + "joint_name", joint_names[i]);

	}
	m_CanCtrlPltf = new NeoCtrlPltfMpo500(&n);
	
	// implementation of topics
	// published topics
	topicPub_JointState = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	
	
	topicPub_Diagnostic = n.advertise<diagnostic_msgs::DiagnosticStatus>("diagnostic", 1);
	// subscribed topics
	topicSub_JointStateCmd = n.subscribe("cmd_joint_traj", 1, &NodeClass::topicCallback_JointStateCmd, this);

}

bool NodeClass::initDrives()
{
	ROS_INFO("Initializing Base Drive Chain");

	// init member vectors
	m_Param.vdWheelNtrlPosRad.assign((m_iNumMotors),0);
	n.getParam("maxDriveRate", m_Param.dMaxDriveRateRadpS);

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
}

void  NodeClass::sendVelCan()
{
	if(m_bisInitialized == true && !bIsError)
	{
		for(int i=0; i<m_iNumMotors; i++){
			m_CanCtrlPltf->setVelGearRadS(i+2, sendVel[i]); //TODO: replace i+2 with can id
			ROS_DEBUG("send velocity to can if: %i : %f", i, sendVel[i]);
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
	
		// check if velocities lie inside allowed boundaries
		for(int i = 0; i < m_iNumMotors; i++)
		{	
			sendVel[i] = msg->points[0].velocities[i];
			if (sendVel[i] > m_Param.dMaxDriveRateRadpS)
			{
				sendVel[i] = m_Param.dMaxDriveRateRadpS;
			}
			if (sendVel[i] < -m_Param.dMaxDriveRateRadpS)
			{
				sendVel[i] = -m_Param.dMaxDriveRateRadpS;
			}
		}
	}
}

// service callback functions
// function will be called when a service is querried

// Init Can-Configuration
bool NodeClass::can_init()
{
	ROS_DEBUG("Service Callback init");
	if(m_bisInitialized == false)
	{
		m_bisInitialized = initDrives();
		//ROS_INFO("...initializing can-nodes...");

		if(m_bisInitialized)
		{
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
bool NodeClass::recover()
{
	if(m_bisInitialized)
	{	
		reset_retries++;
		if(reset_retries%10 == 0)
		{
			usleep(100000);
			ROS_DEBUG("Service callback reset");
			bool reset = m_CanCtrlPltf->resetPltf();
			if (reset) {
				reset_retries = 0;
				ROS_INFO("base resetted");
			} else {
				ROS_WARN("Resetting base failed");
			}
		}
	}
	else
	{
		ROS_WARN("...base already recovered...");
	}

	return true;
}

// shutdown Drivers and Can-Node
bool NodeClass::shutdown()
{
	ROS_DEBUG("Service callback shutdown");
	bool success = m_CanCtrlPltf->shutdownPltf();
	if (success)
		ROS_INFO("Drives shut down");
	else
		ROS_INFO("Shutdown of Drives FAILED");

	return true;
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

			m_CanCtrlPltf->requestMotPosVel(i+2);
			m_CanCtrlPltf->getGearPosVelRadS(i+2,  &vdAngGearRad[i], &vdVelGearRad[i]); //TODO: replace i+2 with canid
			
			//Get motor torque
			if(m_bPubEffort) {
				m_CanCtrlPltf->getMotorTorque(i, &vdEffortGearNM[i]); //(int iCanIdent, double* pdTorqueNm)
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
		bIsError = m_CanCtrlPltf->isPltfError();
		if(bIsError) ROS_ERROR("platform has an error");
	}

	// set data to diagnostics
	if(bIsError)
	{
		diagnostics.level = 2;
		diagnostics.name = "drive-chain can node";
		diagnostics.message = "one or more drives are in Error mode";
		recover();
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
	nodeClass.can_init();

	while(nodeClass.n.ok())
	{

		nodeClass.publish_JointStates();
		nodeClass.sendVelCan();
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}


