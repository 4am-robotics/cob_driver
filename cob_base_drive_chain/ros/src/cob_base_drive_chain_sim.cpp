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
		bool m_bisInitialized;
		bool autoInit;

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
		~NodeClass();

		void topicCallback_JointStateCmd(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
		bool can_init();
		bool srv_can_init(cob_srvs::Trigger::Request& req, cob_srvs::Trigger::Response& res );
		bool recover();
		bool srv_recover(cob_srvs::Trigger::Request& req, cob_srvs::Trigger::Response& res );
		bool shutdown();
		bool srv_shutdown(cob_srvs::Trigger::Request& req, cob_srvs::Trigger::Response& res );
		bool publish_JointStates();
		bool initDrives();

		private:
		std::vector<std::string> joint_names;
		std::vector<int> control_type;
		double *sendVel;
		double *sendAccel;
		double *sendPose;
		int m_iNumMotors;
		bool bIsError;
		ros::Duration timeOut, auto_recover_interval;
		ros::Time last_pub_time;
		bool checkJointNames;
};

//##################################
//#### function implementations ####


// Constructor
NodeClass::NodeClass() : auto_recover_interval(0.3)
{
	// initialization of variables
	m_bisInitialized = false;

	//bIsError = true;
	/// Parameters are set within the launch file
	// Read number of drives from iniFile and pass IniDirectory to CobPlatfCtrl.

	n.param<bool>("PublishEffort", m_bPubEffort, true);
	n.param<bool>("AutoInit",autoInit, true);
	n.param<bool>("CheckJointNames",checkJointNames, false);
	ROS_INFO("autoinitializing base_drive_chain");
	//stop all Drives if no new cmd_vel have been received within $errorStopTime seconds.
	last_pub_time = ros::Time::now();
	double errorStopTime;
	n.param<double>("TimeOut", errorStopTime, 3);
	ros::Duration d(errorStopTime);
	timeOut = d;
	if(m_bPubEffort) ROS_INFO("You have choosen to publish effort of motors, that charges capacity of CAN");

	// get max Joint-Velocities (in rad/s) for Steer- and Drive-Joint
	n.getParam("numberOfMotors", m_iNumMotors);
	sendVel = new double[m_iNumMotors];
	sendAccel = new double[m_iNumMotors];
	sendPose = new double[m_iNumMotors];
	
	for(int i=0; i<m_iNumMotors; i++) sendVel[i] = 0;
	joint_names.resize(m_iNumMotors);
	control_type.resize(m_iNumMotors);
	for(int i=0; i<m_iNumMotors; i++)
	{
		sendVel[i] = 0;
		sendAccel[i] = 0;
		sendPose[i] = 0;
		std::ostringstream stringstream;
		stringstream<<"drive"<<i<<"/";
		std::string pathName = stringstream.str();
		n.getParam(pathName + "joint_name", joint_names[i]);
		n.getParam(pathName + "control_type", control_type[i]);
	}
	
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

bool NodeClass::initDrives()
{
	return true;
}

// Destructor
NodeClass::~NodeClass() 
{
	delete[] sendVel;
	delete[] sendAccel;
	delete[] sendPose;
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
		if(checkJointNames)
		{
			for(int i = 0; i < m_iNumMotors; i++)
			{	
				int id = -1;
				for(int j = 0; j < msg->joint_names.size(); j++)
				{
					if(msg->joint_names[j] == joint_names[i])
					{
						id = j;
						break;
					}
				}
				if(id == -1)
				{
					for(int j = 0; j<m_iNumMotors; j++) sendVel[j] = 0;
					ROS_FATAL("cob_base_drive_chain: unknown joint names in trajectory cmd message");
					return;
				}
				if(msg->points[0].positions.size() > id) sendPose[i] = msg->points[0].positions[id]; 
				if(msg->points[0].velocities.size() > id) sendVel[i] = msg->points[0].velocities[id];
				if(msg->points[0].accelerations.size() > id) sendAccel[i] = msg->points[0].accelerations[id];
			}
		}
		else //assume fixed order
		{
			for(int i = 0; i < m_iNumMotors; i++)
			{	
				if(msg->points[0].positions.size() > i) sendPose[i] = msg->points[0].positions[i]; 
				if(msg->points[0].velocities.size() > i) sendVel[i] = msg->points[0].velocities[i];
				if(msg->points[0].accelerations.size() > i) sendAccel[i] = msg->points[0].accelerations[i];
			}

		}
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
		return true;
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
	ROS_INFO("Drives shut down");
	return true;
}

//publish JointStates cyclical instead of service callback
bool NodeClass::publish_JointStates()
{
	ROS_DEBUG("Service Callback GetJointState");
	// create temporary (local) JointState/Diagnostics Data-Container
	sensor_msgs::JointState jointstate;
	diagnostic_msgs::DiagnosticStatus diagnostics;
	
	// get time stamp for header
	jointstate.header.stamp = ros::Time::now();

	// assign right size to JointState
	jointstate.position.resize(m_iNumMotors);
	jointstate.velocity.resize(m_iNumMotors);
	jointstate.effort.resize(m_iNumMotors);

	if(m_bisInitialized == false)
	{
		ROS_INFO("NOT INTIALIZED");
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
		double dt = (ros::Time::now() - last_pub_time).toSec();
		last_pub_time = ros::Time::now();
		double dt2 = dt * dt;
		// set data to jointstate
		for(int i = 0; i<m_iNumMotors; i++)
		{
			if(control_type[i] == 1) //position control:
			{
				jointstate.position[i] = sendPose[i];
				jointstate.velocity[i] = 0;
				jointstate.effort[i] = 0;
				jointstate.name.push_back(joint_names[i]);
			}
			//if(control_type[i] == 2) velocity control:
			else
			{
				sendPose[i] += sendVel[i] * dt + sendAccel[i] * dt2 / 2;
				sendVel[i] += sendAccel[i] * dt;
				jointstate.position[i] = sendPose[i];
				jointstate.velocity[i] = sendVel[i];
				jointstate.effort[i] = 0;
				jointstate.name.push_back(joint_names[i]);
			}
		}
		
	}


	// publish jointstate message
	topicPub_JointState.publish(jointstate);
	
	// set data to diagnostics
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
	if(nodeClass.autoInit)
	{
		nodeClass.can_init();
	}

	while(nodeClass.n.ok())
	{

		nodeClass.publish_JointStates();
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
