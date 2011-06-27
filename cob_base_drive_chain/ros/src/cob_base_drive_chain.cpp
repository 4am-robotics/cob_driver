/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_base_drive_chain
 * Description: This node provides control of the care-o-bot platform drives to the ROS-"network". For this purpose it offers several services and publishes data on different topics.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Feb 2010:
 * ToDo: Doesn't this node has to take care about the Watchdogs?
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	 * Redistributions of source code must retain the above copyright
 *	   notice, this list of conditions and the following disclaimer.
 *	 * Redistributions in binary form must reproduce the above copyright
 *	   notice, this list of conditions and the following disclaimer in the
 *	   documentation and/or other materials provided with the distribution.
 *	 * Neither the name of the Fraunhofer Institute for Manufacturing 
 *	   Engineering and Automation (IPA) nor the names of its
 *	   contributors may be used to endorse or promote products derived from
 *	   this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

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

// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_base_drive_chain/GetJointState.h>
#include <cob_srvs/ElmoRecorderReadout.h>
#include <cob_srvs/ElmoRecorderConfig.h>


// external includes
#include <cob_base_drive_chain/CanCtrlPltfCOb3.h>
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

		// topics to publish
		/**
		* On this topic "JointState" of type sensor_msgs::JointState the node publishes joint states when they are requested over the appropriate service srvServer_GetJointState.
		*/
		ros::Publisher topicPub_JointState;

		ros::Publisher topicPub_ControllerState;


		/**
		* On this topic "Diagnostic" of type diagnostic_msgs::DiagnosticStatus the node publishes states and error information about the platform.
		*/
		ros::Publisher topicPub_Diagnostic;


		// topics to subscribe, callback is called for new messages arriving
		/**
		* The node subscribes to the topic "JointStateCmd" and performs the requested motor commands
		*/
		ros::Subscriber topicSub_JointStateCmd;

		// service servers
		/**
		* Service requests cob_srvs::Trigger and initializes platform and motors
		*/
		ros::ServiceServer srvServer_Init;

		/**
		* Service requests cob_srvs::Trigger and resets platform and motors
		*/
		ros::ServiceServer srvServer_Recover;

		/**
		* Service requests cob_srvs::Trigger and shuts down platform and motors
		*/
		ros::ServiceServer srvServer_Shutdown;

		ros::ServiceServer srvServer_SetMotionType;

		/**
		* Service requests cob_srvs::GetJointState. It reads out the latest joint information from the CAN buffer and gives it back. It also publishes the informaion on the topic "JointState"
		*/
		//ros::ServiceServer srvServer_GetJointState;

		/**
		* Service requests cob_srvs::ElmoRecorderSetup. It is used to configure the Elmo Recorder to record predefined sources. 
		* Parameters are:
		* int64 recordinggap #Specify every which time quantum (4*90usec) a new data point (of 1024 points in total) is recorded. the recording process starts immediately.
		*/
		ros::ServiceServer srvServer_ElmoRecorderConfig;

		/**
		* Service requests cob_srvs::ElmoRecorderGet. It is used to start the read-out process of previously recorded data by the Elmo Recorder.
		* Parameters are:
		* int64 subindex 
		* #As Subindex, set the recorded source you want to read out:
		* #1: Main Speed
		* #2: Main Position
		* #10: ActiveCurrent
		* #16: Speed Command
		*
		* string fileprefix
		* #Enter the path+file-prefix for the logfile (of an existing directory!)
		* #The file-prefix is extended with _MotorNumber_RecordedSource.log
		*/
		ros::ServiceServer srvServer_ElmoRecorderReadout;

		// global variables
		// generate can-node handle
#ifdef __SIM__
		ros::Publisher br_steer_pub;
		ros::Publisher bl_steer_pub;
		ros::Publisher fr_steer_pub;
		ros::Publisher fl_steer_pub;
		ros::Publisher br_caster_pub;
		ros::Publisher bl_caster_pub;
		ros::Publisher fr_caster_pub;
		ros::Publisher fl_caster_pub;
		
		ros::Subscriber br_steer_sub;
		ros::Subscriber bl_steer_sub;
		ros::Subscriber fr_steer_sub;
		ros::Subscriber fl_steer_sub;
		ros::Subscriber br_caster_sub;
		ros::Subscriber bl_caster_sub;
		ros::Subscriber fr_caster_sub;
		ros::Subscriber fl_caster_sub;
		
		std::vector<double> m_gazeboPos;
		std::vector<double> m_gazeboVel;
#else
		CanCtrlPltfCOb3 *m_CanCtrlPltf;
#endif
		bool m_bisInitialized;
		int m_iNumMotors;
		int m_iNumDrives;

		struct ParamType
		{ 
			double dMaxDriveRateRadpS;
			double dMaxSteerRateRadpS;

			std::vector<double> vdWheelNtrlPosRad;
		};
		ParamType m_Param;
		
		std::string sIniDirectory;
		bool m_bPubEffort;
		bool m_bReadoutElmo;

		// Constructor
		NodeClass()
		{
			// initialization of variables
#ifdef __SIM__
			m_bisInitialized = initDrives();
#else
			m_bisInitialized = false;
#endif

			/// Parameters are set within the launch file
			// Read number of drives from iniFile and pass IniDirectory to CobPlatfCtrl.
			if (n.hasParam("IniDirectory"))
			{
				n.getParam("IniDirectory", sIniDirectory);
				ROS_INFO("IniDirectory loaded from Parameter-Server is: %s", sIniDirectory.c_str());
			}
			else
			{
				sIniDirectory = "Platform/IniFiles/";
				ROS_WARN("IniDirectory not found on Parameter-Server, using default value: %s", sIniDirectory.c_str());
			}

			n.param<bool>("PublishEffort", m_bPubEffort, false);
			if(m_bPubEffort) ROS_INFO("You have choosen to publish effort of motors, that charges capacity of CAN");
			
			
			IniFile iniFile;
			iniFile.SetFileName(sIniDirectory + "Platform.ini", "PltfHardwareCoB3.h");

			// get max Joint-Velocities (in rad/s) for Steer- and Drive-Joint
			iniFile.GetKeyInt("Config", "NumberOfMotors", &m_iNumMotors, true);
			iniFile.GetKeyInt("Config", "NumberOfWheels", &m_iNumDrives, true);
			if(m_iNumMotors < 2 || m_iNumMotors > 8) {
				m_iNumMotors = 8;
				m_iNumDrives = 4;
			}
			
#ifdef __SIM__
			bl_caster_pub = n.advertise<std_msgs::Float64>("/base_bl_caster_r_wheel_controller/command", 1);
			br_caster_pub = n.advertise<std_msgs::Float64>("/base_br_caster_r_wheel_controller/command", 1);
			fl_caster_pub = n.advertise<std_msgs::Float64>("/base_fl_caster_r_wheel_controller/command", 1);
			fr_caster_pub = n.advertise<std_msgs::Float64>("/base_fr_caster_r_wheel_controller/command", 1);
			bl_steer_pub = n.advertise<std_msgs::Float64>("/base_bl_caster_rotation_controller/command", 1);
			br_steer_pub = n.advertise<std_msgs::Float64>("/base_br_caster_rotation_controller/command", 1);
			fl_steer_pub = n.advertise<std_msgs::Float64>("/base_fl_caster_rotation_controller/command", 1);
			fr_steer_pub = n.advertise<std_msgs::Float64>("/base_fr_caster_rotation_controller/command", 1);

			bl_caster_sub = n.subscribe("/base_bl_caster_r_wheel_controller/state", 1, &NodeClass::gazebo_bl_caster_Callback, this);
			br_caster_sub = n.subscribe("/base_br_caster_r_wheel_controller/state", 1, &NodeClass::gazebo_br_caster_Callback, this);
			fl_caster_sub = n.subscribe("/base_fl_caster_r_wheel_controller/state", 1, &NodeClass::gazebo_fl_caster_Callback, this);
			fr_caster_sub = n.subscribe("/base_fr_caster_r_wheel_controller/state", 1, &NodeClass::gazebo_fr_caster_Callback, this);
			bl_steer_sub = n.subscribe("/base_bl_caster_rotation_controller/state", 1, &NodeClass::gazebo_bl_steer_Callback, this);
			br_steer_sub = n.subscribe("/base_br_caster_rotation_controller/state", 1, &NodeClass::gazebo_br_steer_Callback, this);
			fl_steer_sub = n.subscribe("/base_fl_caster_rotation_controller/state", 1, &NodeClass::gazebo_fl_steer_Callback, this);
			fr_steer_sub = n.subscribe("/base_fr_caster_rotation_controller/state", 1, &NodeClass::gazebo_fr_steer_Callback, this);
			
			m_gazeboPos.resize(m_iNumMotors);
			m_gazeboVel.resize(m_iNumMotors);
#else
			m_CanCtrlPltf = new CanCtrlPltfCOb3(sIniDirectory);
#endif
			
			// implementation of topics
			// published topics
			topicPub_JointState = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
			topicPub_ControllerState = n.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("state", 1);
			
			
			topicPub_Diagnostic = n.advertise<diagnostic_msgs::DiagnosticStatus>("diagnostic", 1);
			// subscribed topics
			topicSub_JointStateCmd = n.subscribe("joint_command", 1, &NodeClass::topicCallback_JointStateCmd, this);

			// implementation of service servers
			srvServer_Init = n.advertiseService("init", &NodeClass::srvCallback_Init, this);
			srvServer_ElmoRecorderConfig = n.advertiseService("ElmoRecorderConfig", &NodeClass::srvCallback_ElmoRecorderConfig, this);
			srvServer_ElmoRecorderReadout = n.advertiseService("ElmoRecorderReadout", &NodeClass::srvCallback_ElmoRecorderReadout, this);
			m_bReadoutElmo = false;

			srvServer_Recover = n.advertiseService("recover", &NodeClass::srvCallback_Recover, this);
			srvServer_Shutdown = n.advertiseService("shutdown", &NodeClass::srvCallback_Shutdown, this);
			//srvServer_isPltfError = n.advertiseService("isPltfError", &NodeClass::srvCallback_isPltfError, this); --> Publish this along with JointStates
			//srvServer_GetJointState = n.advertiseService("GetJointState", &NodeClass::srvCallback_GetJointState, this);
		}

		// Destructor
		~NodeClass() 
		{
#ifdef __SIM__

#else
			m_CanCtrlPltf->shutdownPltf();
#endif
		}

		// topic callback functions 
		// function will be called when a new message arrives on a topic
		void topicCallback_JointStateCmd(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg)
		{
			ROS_DEBUG("Topic Callback joint_command");
			// only process cmds when system is initialized
			if(m_bisInitialized == true)
			{
				ROS_DEBUG("Topic Callback joint_command - Sending Commands to drives (initialized)");
				sensor_msgs::JointState JointStateCmd;
				JointStateCmd.position.resize(m_iNumMotors);
				JointStateCmd.velocity.resize(m_iNumMotors);
				JointStateCmd.effort.resize(m_iNumMotors);
				
				for(unsigned int i = 0; i < msg->joint_names.size(); i++)
				{
					// associate inputs to according steer and drive joints
					// ToDo: specify this globally (Prms-File or config-File or via msg-def.)
					// check if velocities lie inside allowed boundaries
					
					//DRIVES
					if(msg->joint_names[i] ==  "fl_caster_r_wheel_joint")
					{
							JointStateCmd.position[0] = msg->desired.positions[i];
							JointStateCmd.velocity[0] = msg->desired.velocities[i];
							//JointStateCmd.effort[0] = msg->effort[i];
					}
					if(msg->joint_names[i] ==  "bl_caster_r_wheel_joint")
					{
							JointStateCmd.position[2] = msg->desired.positions[i];
							JointStateCmd.velocity[2] = msg->desired.velocities[i];
							//JointStateCmd.effort[2] = msg->effort[i];
					}
					if(msg->joint_names[i] ==  "br_caster_r_wheel_joint")
					{
							JointStateCmd.position[4] = msg->desired.positions[i];
							JointStateCmd.velocity[4] = msg->desired.velocities[i];
							//JointStateCmd.effort[4] = msg->effort[i];
					}
					if(msg->joint_names[i] ==  "fr_caster_r_wheel_joint")
					{
							JointStateCmd.position[6] = msg->desired.positions[i];
							JointStateCmd.velocity[6] = msg->desired.velocities[i];
							//JointStateCmd.effort[6] = msg->effort[i];
					}
					//STEERS
					if(msg->joint_names[i] ==  "fl_caster_rotation_joint")
					{
							JointStateCmd.position[1] = msg->desired.positions[i];
							JointStateCmd.velocity[1] = msg->desired.velocities[i];
							//JointStateCmd.effort[1] = msg->effort[i];
					}
					if(msg->joint_names[i] ==  "bl_caster_rotation_joint")
					{ 
							JointStateCmd.position[3] = msg->desired.positions[i];
							JointStateCmd.velocity[3] = msg->desired.velocities[i];
							//JointStateCmd.effort[3] = msg->effort[i];
					}
					if(msg->joint_names[i] ==  "br_caster_rotation_joint")
					{
							JointStateCmd.position[5] = msg->desired.positions[i];
							JointStateCmd.velocity[5] = msg->desired.velocities[i];
							//JointStateCmd.effort[5] = msg->effort[i];
					}
					if(msg->joint_names[i] ==  "fr_caster_rotation_joint")
					{
							JointStateCmd.position[7] = msg->desired.positions[i];
							JointStateCmd.velocity[7] = msg->desired.velocities[i];
							//JointStateCmd.effort[7] = msg->effort[i];
					}
			
				}
				
				
				// check if velocities lie inside allowed boundaries
				for(int i = 0; i < m_iNumMotors; i++)
				{
					// for steering motors
					if( i == 1 || i == 3 || i == 5 || i == 7) // ToDo: specify this via the config-files
					{
						if (JointStateCmd.velocity[i] > m_Param.dMaxSteerRateRadpS)
						{
							JointStateCmd.velocity[i] = m_Param.dMaxSteerRateRadpS;
						}
						if (JointStateCmd.velocity[i] < -m_Param.dMaxSteerRateRadpS)
						{
							JointStateCmd.velocity[i] = -m_Param.dMaxSteerRateRadpS;
						}
					}
					// for driving motors
					else
					{
						if (JointStateCmd.velocity[i] > m_Param.dMaxDriveRateRadpS)
						{
							JointStateCmd.velocity[i] = m_Param.dMaxDriveRateRadpS;
						}
						if (JointStateCmd.velocity[i] < -m_Param.dMaxDriveRateRadpS)
						{
							JointStateCmd.velocity[i] = -m_Param.dMaxDriveRateRadpS;
						}
					}

					// and cmd velocities to Can-Nodes
					//m_CanCtrlPltf->setVelGearRadS(iCanIdent, dVelEncRadS);
#ifdef __SIM__
					ROS_DEBUG("Send velocity data to gazebo");
					std_msgs::Float64 fl;
					fl.data = JointStateCmd.velocity[i];
					if(msg->joint_names[i] == "fl_caster_r_wheel_joint")
						fl_caster_pub.publish(fl);
					if(msg->joint_names[i] == "fr_caster_r_wheel_joint")
						fr_caster_pub.publish(fl);
					if(msg->joint_names[i] == "bl_caster_r_wheel_joint")
						bl_caster_pub.publish(fl);
					if(msg->joint_names[i] == "br_caster_r_wheel_joint")
						br_caster_pub.publish(fl);

					if(msg->joint_names[i] == "fl_caster_rotation_joint")
						fl_steer_pub.publish(fl);
					if(msg->joint_names[i] == "fr_caster_rotation_joint")
						fr_steer_pub.publish(fl);
					if(msg->joint_names[i] == "bl_caster_rotation_joint")
						bl_steer_pub.publish(fl);
					if(msg->joint_names[i] == "br_caster_rotation_joint")
						br_steer_pub.publish(fl);
					ROS_DEBUG("Successfully sent velicities to gazebo");
#else
					ROS_DEBUG("Send velocity data to drives");
					m_CanCtrlPltf->setVelGearRadS(i, JointStateCmd.velocity[i]);
					ROS_DEBUG("Successfully sent velicities to drives");
#endif
					
					if(m_bPubEffort)
					{
#ifdef __SIM__

#else
						m_CanCtrlPltf->requestMotorTorque();
#endif
					}
	  			}
			}
		}

		// service callback functions
		// function will be called when a service is querried

		// Init Can-Configuration
		bool srvCallback_Init(cob_srvs::Trigger::Request &req,
							  cob_srvs::Trigger::Response &res )
		{
			ROS_DEBUG("Service Callback init");
			if(m_bisInitialized == false)
			{
				m_bisInitialized = initDrives();
				//ROS_INFO("...initializing can-nodes...");
				//m_bisInitialized = m_CanCtrlPltf->initPltf();
				res.success.data = m_bisInitialized;
				if(m_bisInitialized)
				{
		   			ROS_INFO("base initialized");
				}
				else
				{
					res.error_message.data = "initialization of base failed";
				  	ROS_ERROR("Initializing base failed");
				}
			}
			else
			{
				ROS_WARN("...base already initialized...");
				res.success.data = true;
				res.error_message.data = "platform already initialized";
			}
			return true;
		}
		
		bool srvCallback_ElmoRecorderConfig(cob_srvs::ElmoRecorderConfig::Request &req,
							  cob_srvs::ElmoRecorderConfig::Response &res ){
			if(m_bisInitialized) 
			{
#ifdef __SIM__
				res.success = true;
#else
				m_CanCtrlPltf->evalCanBuffer();
				res.success = m_CanCtrlPltf->ElmoRecordings(0, req.recordinggap, "");
#endif
				res.message = "Successfully configured all motors for instant record";
			}

			return true;
		}
		
		bool srvCallback_ElmoRecorderReadout(cob_srvs::ElmoRecorderReadout::Request &req,
							  cob_srvs::ElmoRecorderReadout::Response &res ){
			if(m_bisInitialized) {
#ifdef __SIM__
				res.success = true;
#else
				m_CanCtrlPltf->evalCanBuffer();
				res.success = m_CanCtrlPltf->ElmoRecordings(1, req.subindex, req.fileprefix);
#endif
				if(res.success == 0) {
					res.message = "Successfully requested reading out of Recorded data";
					m_bReadoutElmo = true;
					ROS_WARN("CPU consuming evalCanBuffer used for ElmoReadout activated");
				} else if(res.success == 1) res.message = "Recorder hasn't been configured well yet";
				else if(res.success == 2) res.message = "A previous transmission is still in progress";
			}

			return true;
		}
		
		
		
		// reset Can-Configuration
		bool srvCallback_Recover(cob_srvs::Trigger::Request &req,
									 cob_srvs::Trigger::Response &res )
		{
			if(m_bisInitialized)
			{
				ROS_DEBUG("Service callback reset");
#ifdef __SIM__
				res.success.data = true;
#else
				res.success.data = m_CanCtrlPltf->resetPltf();
#endif
				if (res.success.data) {
		   			ROS_INFO("base resetted");
				} else {
					res.error_message.data = "reset of base failed";
					ROS_WARN("Resetting base failed");
				}
			}
			else
			{
				ROS_WARN("...base already recovered...");
				res.success.data = true;
				res.error_message.data = "base already recovered";
			}

			return true;
		}
		
		// shutdown Drivers and Can-Node
		bool srvCallback_Shutdown(cob_srvs::Trigger::Request &req,
									 cob_srvs::Trigger::Response &res )
		{
			ROS_DEBUG("Service callback shutdown");
#ifdef __SIM__
			res.success.data = true;
#else
			res.success.data = m_CanCtrlPltf->shutdownPltf();
#endif
			if (res.success.data)
	   			ROS_INFO("Drives shut down");
			else
	   			ROS_INFO("Shutdown of Drives FAILED");

			return true;
		}


		/*
		bool srvCallback_GetJointState(cob_base_drive_chain::GetJointState::Request &req,
									 cob_base_drive_chain::GetJointState::Response &res )
		{
			ROS_DEBUG("Service Callback GetJointState");
			// init local variables
			int j, k, ret_sprintf;
			bool bIsError;
			std::vector<double> vdAngGearRad, vdVelGearRad, vdEffortGearNM;
			std::string str_steer, str_drive, str_num, str_cat;
			// ToDo: search for a more elegant way to compose JointNames
			char c_num [1];

			// init strings
			str_steer = "Steer";
			str_drive = "Drive";

			// set default values
			vdAngGearRad.resize(m_iNumMotors, 0);
			vdVelGearRad.resize(m_iNumMotors, 0);
			vdEffortGearNM.resize(m_iNumMotors, 0);

			// create temporary (local) JointState/Diagnostics Data-Container
			sensor_msgs::JointState jointstate;
			diagnostic_msgs::DiagnosticStatus diagnostics;
			

			//Do you have to set frame_id manually??

			// get time stamp for header
			jointstate.header.stamp = ros::Time::now();
			// set frame_id for header
			// jointstate.header.frame_id = frame_id; //Where to get this id from?

			// assign right size to JointState
			jointstate.name.resize(m_iNumMotors);
			jointstate.position.resize(m_iNumMotors);
			jointstate.velocity.resize(m_iNumMotors);
			jointstate.effort.resize(m_iNumMotors);

			if(m_bisInitialized == false)
			{
				// as long as system is not initialized
				bIsError = false;

				j = 0;
				k = 0;

				// set data to jointstate			
				for(int i = 0; i<m_iNumMotors; i++)
				{
					jointstate.position[i] = 0.0;
					jointstate.velocity[i] = 0.0;
					jointstate.effort[i] = 0.0;

					// set joint names
   					if( i == 1 || i == 3 || i == 5 || i == 7) // ToDo: specify this via the config-files
					{
						// create name for identification in JointState msg
						j = j+1;
						ret_sprintf = sprintf(c_num, "%i", j);
						str_num.assign(1, c_num[0]);
						str_cat = str_steer + str_num;
					}
					else
					{
						// create name for identification in JointState msg
						k = k+1;
						ret_sprintf = sprintf(c_num, "%i", k);
						str_num.assign(1, c_num[0]);
						str_cat = str_drive + str_num;
					}
					// set joint names
					jointstate.name[i] = str_cat;
				}
			}
			else
			{
				// as soon as drive chain is initialized
				// read Can-Buffer
				m_CanCtrlPltf->evalCanBuffer();
				
				j = 0;
				k = 0;
				for(int i = 0; i<m_iNumMotors; i++)
				{
					m_CanCtrlPltf->getGearPosVelRadS(i,  &vdAngGearRad[i], &vdVelGearRad[i]);
					
					//Get motor torque
					if(m_bPubEffort) {
						for(int i=0; i<m_iNumMotors; i++) {
							m_CanCtrlPltf->getMotorTorque(i, &vdEffortGearNM[i]); //(int iCanIdent, double* pdTorqueNm)
						}
					}
					
   					// if a steering motor was read -> correct for offset
   					if( i == 1 || i == 3 || i == 5 || i == 7) // ToDo: specify this via the config-files
					{
						// correct for initial offset of steering angle (arbitrary homing position)
						vdAngGearRad[i] += m_Param.vdWheelNtrlPosRad[j];
						MathSup::normalizePi(vdAngGearRad[i]);
						j = j+1;
						// create name for identification in JointState msg
						ret_sprintf = sprintf(c_num, "%i", j);
						str_num.assign(1, c_num[0]);
						str_cat = str_steer + str_num;
					}
					else
					{
						// create name for identification in JointState msg
						k = k+1;
						ret_sprintf = sprintf(c_num, "%i", k);
						str_num.assign(1, c_num[0]);
						str_cat = str_drive + str_num;
					}
					// set joint names
					jointstate.name[i] = str_cat;
				}

				// set data to jointstate
				for(int i = 0; i<m_iNumMotors; i++)
				{
					jointstate.position[i] = vdAngGearRad[i];
					jointstate.velocity[i] = vdVelGearRad[i];
					jointstate.effort[i] = vdEffortGearNM[i];
				}
			}

			// set answer to srv request
			res.jointstate = jointstate;

			// publish jointstate message
			topicPub_JointState.publish(jointstate);
			ROS_DEBUG("published new drive-chain configuration (JointState message)");
			

			if(m_bisInitialized)
			{
				// read Can only after initialization
				bIsError = m_CanCtrlPltf->isPltfError();
			}

			// set data to diagnostics
			if(bIsError)
			{
				diagnostics.level = 2;
				diagnostics.name = "drive-chain can node";
				diagnostics.message = "one or more drives are in Error mode";
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
		*/

		//publish JointStates cyclical instead of service callback
		bool publish_JointStates()
		{
			ROS_DEBUG("Service Callback GetJointState");
			// init local variables
			int j, k;
			bool bIsError;
			std::vector<double> vdAngGearRad, vdVelGearRad, vdEffortGearNM;

			// set default values
			vdAngGearRad.resize(m_iNumMotors, 0);
			vdVelGearRad.resize(m_iNumMotors, 0);
			vdEffortGearNM.resize(m_iNumMotors, 0);

			// create temporary (local) JointState/Diagnostics Data-Container
			sensor_msgs::JointState jointstate;
			diagnostic_msgs::DiagnosticStatus diagnostics;
			
			pr2_controllers_msgs::JointTrajectoryControllerState controller_state;
			

			//Do you have to set frame_id manually??

			// get time stamp for header
			jointstate.header.stamp = ros::Time::now();
			// set frame_id for header
			// jointstate.header.frame_id = frame_id; //Where to get this id from?

			// assign right size to JointState
			
			//jointstate.name.resize(m_iNumMotors);
			jointstate.position.assign(m_iNumMotors, 0.0);
			jointstate.velocity.assign(m_iNumMotors, 0.0);
			jointstate.effort.assign(m_iNumMotors, 0.0);

			if(m_bisInitialized == false)
			{
				// as long as system is not initialized
				bIsError = false;

				j = 0;
				k = 0;

				// set data to jointstate			
				for(int i = 0; i<m_iNumMotors; i++)
				{
					jointstate.position[i] = 0.0;
					jointstate.velocity[i] = 0.0;
					jointstate.effort[i] = 0.0;
				}
				jointstate.name.push_back("fl_caster_r_wheel_joint");
				jointstate.name.push_back("fl_caster_rotation_joint");
				jointstate.name.push_back("bl_caster_r_wheel_joint");
				jointstate.name.push_back("bl_caster_rotation_joint");
				jointstate.name.push_back("br_caster_r_wheel_joint");
				jointstate.name.push_back("br_caster_rotation_joint");
				jointstate.name.push_back("fr_caster_r_wheel_joint");
				jointstate.name.push_back("fr_caster_rotation_joint");
			
			}
			else
			{
				// as soon as drive chain is initialized
				// read Can-Buffer
				ROS_DEBUG("Read CAN-Buffer");
#ifdef __SIM__

#else
				m_CanCtrlPltf->evalCanBuffer();
#endif
				ROS_DEBUG("Successfully read CAN-Buffer");
				
				j = 0;
				k = 0;
				for(int i = 0; i<m_iNumMotors; i++)
				{
#ifdef __SIM__
					vdAngGearRad[i] = m_gazeboPos[i];
					vdVelGearRad[i] = m_gazeboVel[i];
#else
					m_CanCtrlPltf->getGearPosVelRadS(i,  &vdAngGearRad[i], &vdVelGearRad[i]);
#endif
					
					//Get motor torque
					if(m_bPubEffort) {
						for(int i=0; i<m_iNumMotors; i++) 
						{
#ifdef __SIM__
							//vdEffortGearNM[i] = m_gazeboEff[i];
#else
							m_CanCtrlPltf->getMotorTorque(i, &vdEffortGearNM[i]); //(int iCanIdent, double* pdTorqueNm)
#endif
						}
					}
					
   					// if a steering motor was read -> correct for offset
   					if( i == 1 || i == 3 || i == 5 || i == 7) // ToDo: specify this via the config-files
					{
						// correct for initial offset of steering angle (arbitrary homing position)
						vdAngGearRad[i] += m_Param.vdWheelNtrlPosRad[j];
						MathSup::normalizePi(vdAngGearRad[i]);
						j = j+1;
					}

				}

				// set data to jointstate
				for(int i = 0; i<m_iNumMotors; i++)
				{
					jointstate.position[i] = vdAngGearRad[i];
					jointstate.velocity[i] = vdVelGearRad[i];
					jointstate.effort[i] = vdEffortGearNM[i];
				}
				
				jointstate.name.push_back("fl_caster_r_wheel_joint");
				jointstate.name.push_back("fl_caster_rotation_joint");
				jointstate.name.push_back("bl_caster_r_wheel_joint");
				jointstate.name.push_back("bl_caster_rotation_joint");
				jointstate.name.push_back("br_caster_r_wheel_joint");
				jointstate.name.push_back("br_caster_rotation_joint");
				jointstate.name.push_back("fr_caster_r_wheel_joint");
				jointstate.name.push_back("fr_caster_rotation_joint");
				
			}

			controller_state.joint_names = jointstate.name;
			controller_state.actual.positions = jointstate.position;
			controller_state.actual.velocities = jointstate.velocity;

			// publish jointstate message
			topicPub_JointState.publish(jointstate);
			topicPub_ControllerState.publish(controller_state);
			
			ROS_DEBUG("published new drive-chain configuration (JointState message)");
			

			if(m_bisInitialized)
			{
				// read Can only after initialization
#ifdef __SIM__
				bIsError = false;
#else
				bIsError = m_CanCtrlPltf->isPltfError();
#endif
			}

			// set data to diagnostics
			if(bIsError)
			{
				diagnostics.level = 2;
				diagnostics.name = "drive-chain can node";
				diagnostics.message = "one or more drives are in Error mode";
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
		
		// other function declarations
		bool initDrives();

#ifdef __SIM__
		// get pos and vel values for drives and steers from gazebo

		// DRIVES
		// fl_caster_r_wheel_joint is JointStateCmd[0]
		void gazebo_fl_caster_Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
		{
			m_gazeboPos[0] = msg->process_value;
			m_gazeboVel[0] = msg->process_value_dot;
		}
		// bl_caster_r_wheel_joint is JointStateCmd[2]
		void gazebo_bl_caster_Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
		{
			m_gazeboPos[2] = msg->process_value;
			m_gazeboVel[2] = msg->process_value_dot;
		}
		// br_caster_r_wheel_joint is JointStateCmd[4]
		void gazebo_br_caster_Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
		{
			m_gazeboPos[4] = msg->process_value;
			m_gazeboVel[4] = msg->process_value_dot;
		}
		// fr_caster_r_wheel_joint is JointStateCmd[6]
		void gazebo_fr_caster_Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
		{
			m_gazeboPos[6] = msg->process_value;
			m_gazeboVel[6] = msg->process_value_dot;
		}

		// STEERS		
		// fl_caster_rotation_joint is JointStateCmd[1]
		void gazebo_fl_steer_Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
		{
			m_gazeboPos[1] = msg->process_value;
			m_gazeboVel[1] = msg->process_value_dot;
		}
		// bl_caster_rotation_joint is JointStateCmd[3]
		void gazebo_bl_steer_Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
		{
			m_gazeboPos[3] = msg->process_value;
			m_gazeboVel[3] = msg->process_value_dot;
		}
		// br_caster_rotation_joint is JointStateCmd[5]
		void gazebo_br_steer_Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
		{
			m_gazeboPos[5] = msg->process_value;
			m_gazeboVel[5] = msg->process_value_dot;
		}
		// fr_caster_rotation_joint is JointStateCmd[7]
		void gazebo_fr_steer_Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
		{
			m_gazeboPos[7] = msg->process_value;
			m_gazeboVel[7] = msg->process_value_dot;
		}
#else

#endif
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "base_drive_chain");

	NodeClass nodeClass;

	// specify looprate of control-cycle
 	ros::Rate loop_rate(100); // Hz 

	while(nodeClass.n.ok())
	{
#ifdef __SIM__

#else
		//Read-out of CAN buffer is only necessary during read-out of Elmo Recorder		
		if( nodeClass.m_bReadoutElmo ) 
		{
			if(nodeClass.m_bisInitialized) nodeClass.m_CanCtrlPltf->evalCanBuffer();
			if(nodeClass.m_CanCtrlPltf->ElmoRecordings(100, 0, "") == 0)
			{
				nodeClass.m_bReadoutElmo = false;
				ROS_INFO("CPU consuming evalCanBuffer used for ElmoReadout deactivated");
			}
		}
#endif

		nodeClass.publish_JointStates();
		
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

//##################################
//#### function implementations ####
bool NodeClass::initDrives()
{
	ROS_INFO("Initializing Base Drive Chain");

	// init member vectors
	m_Param.vdWheelNtrlPosRad.assign((m_iNumDrives),0);
//	m_Param.vdWheelNtrlPosRad.assign(4,0);
	// ToDo: replace the following steps by ROS configuration files
	// create Inifile class and set target inifile (from which data shall be read)
	IniFile iniFile;

	//n.param<std::string>("PltfIniLoc", sIniFileName, "Platform/IniFiles/Platform.ini");
	iniFile.SetFileName(sIniDirectory + "Platform.ini", "PltfHardwareCoB3.h");

	// get max Joint-Velocities (in rad/s) for Steer- and Drive-Joint
	iniFile.GetKeyDouble("DrivePrms", "MaxDriveRate", &m_Param.dMaxDriveRateRadpS, true);
	iniFile.GetKeyDouble("DrivePrms", "MaxSteerRate", &m_Param.dMaxSteerRateRadpS, true);
	
	// get Offset from Zero-Position of Steering
	if(m_iNumDrives >=1)
		iniFile.GetKeyDouble("DrivePrms", "Wheel1NeutralPosition", &m_Param.vdWheelNtrlPosRad[0], true);
	if(m_iNumDrives >=2)
		iniFile.GetKeyDouble("DrivePrms", "Wheel2NeutralPosition", &m_Param.vdWheelNtrlPosRad[1], true);
	if(m_iNumDrives >=3)
		iniFile.GetKeyDouble("DrivePrms", "Wheel3NeutralPosition", &m_Param.vdWheelNtrlPosRad[2], true);
	if(m_iNumDrives >=4)
		iniFile.GetKeyDouble("DrivePrms", "Wheel4NeutralPosition", &m_Param.vdWheelNtrlPosRad[3], true);

	//Convert Degree-Value from ini-File into Radian:
	for(int i=0; i<m_iNumDrives; i++)
	{
		m_Param.vdWheelNtrlPosRad[i] = MathSup::convDegToRad(m_Param.vdWheelNtrlPosRad[i]);
	}
//	m_Param.vdWheelNtrlPosRad[1] = MathSup::convDegToRad(m_Param.vdWheelNtrlPosRad[1]);
//	m_Param.vdWheelNtrlPosRad[2] = MathSup::convDegToRad(m_Param.vdWheelNtrlPosRad[2]);
//	m_Param.vdWheelNtrlPosRad[3] = MathSup::convDegToRad(m_Param.vdWheelNtrlPosRad[3]);

	// debug log
	ROS_INFO("Initializing CanCtrlItf");
	bool bTemp1;
#ifdef __SIM__
	bTemp1 = true;
#else
	bTemp1 =  m_CanCtrlPltf->initPltf();
#endif
	// debug log
	ROS_INFO("Initializing done");


	return bTemp1;
}
