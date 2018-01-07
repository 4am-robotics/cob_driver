/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


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
#include <diagnostic_msgs/DiagnosticArray.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/JointControllerState.h>

// ROS service includes
#include <std_srvs/Trigger.h>
#include <cob_base_drive_chain/ElmoRecorderReadout.h>
#include <cob_base_drive_chain/ElmoRecorderConfig.h>


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
		* On this topic "JointState" of type sensor_msgs::JointState the node publishes joint states
		*/
		ros::Publisher topicPub_JointState;

		ros::Publisher topicPub_ControllerState;
		ros::Publisher topicPub_DiagnosticGlobal_;


		/**
		* On this topic "Diagnostic" of type diagnostic_msgs::DiagnosticStatus the node publishes states and error information about the platform.
		*/
		ros::Publisher topicPub_Diagnostic;

                /**
                * Timer to publish global diagnostic messages
                */
		ros::Timer glDiagnostics_timer;

		// topics to subscribe, callback is called for new messages arriving
		/**
		* The node subscribes to the topic "JointStateCmd" and performs the requested motor commands
		*/
		ros::Subscriber topicSub_JointStateCmd;

		// service servers
		/**
		* Service requests std_srvs::Trigger and initializes platform and motors
		*/
		ros::ServiceServer srvServer_Init;

		/**
		* Service requests std_srvs::Trigger and resets platform and motors
		*/
		ros::ServiceServer srvServer_Recover;

		/**
		* Service requests std_srvs::Trigger and shuts down platform and motors
		*/
		ros::ServiceServer srvServer_Shutdown;

		ros::ServiceServer srvServer_SetMotionType;

		/**
		* Service requests cob_base_drive_chain::ElmoRecorderSetup. It is used to configure the Elmo Recorder to record predefined sources.
		* Parameters are:
		* int64 recordinggap #Specify every which time quantum (4*90usec) a new data point (of 1024 points in total) is recorded. the recording process starts immediately.
		*/
		ros::ServiceServer srvServer_ElmoRecorderConfig;

		/**
		* Service requests cob_base_drive_chain::ElmoRecorderGet. It is used to start the read-out process of previously recorded data by the Elmo Recorder.
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

		std::vector<double> m_gazeboPos;
		std::vector<double> m_gazeboVel;
		ros::Time m_gazeboStamp;

		ros::Subscriber topicSub_GazeboJointStates;
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

			topicSub_GazeboJointStates = n.subscribe("/joint_states", 1, &NodeClass::gazebo_joint_states_Callback, this);

			m_gazeboPos.resize(m_iNumMotors);
			m_gazeboVel.resize(m_iNumMotors);
#else
			topicPub_JointState = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
			m_CanCtrlPltf = new CanCtrlPltfCOb3(sIniDirectory);
#endif

			// implementation of topics
			// published topics
			topicPub_ControllerState = n.advertise<control_msgs::JointTrajectoryControllerState>("state", 1);
			topicPub_DiagnosticGlobal_ = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

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

		        //Timer for publishing global diagnostics
		        glDiagnostics_timer = n.createTimer(ros::Duration(1), &NodeClass::publish_globalDiagnostics, this);

			// initialization of variables
#ifdef __SIM__
			m_bisInitialized = initDrives();
#else
			m_bisInitialized = false;
#endif

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
		void topicCallback_JointStateCmd(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
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
					else if(m_iNumDrives>=2 && msg->joint_names[i] ==  "bl_caster_r_wheel_joint")
					{
							JointStateCmd.position[2] = msg->desired.positions[i];
							JointStateCmd.velocity[2] = msg->desired.velocities[i];
							//JointStateCmd.effort[2] = msg->effort[i];
					}
					else if(m_iNumDrives>=3 && msg->joint_names[i] ==  "br_caster_r_wheel_joint")
					{
							JointStateCmd.position[4] = msg->desired.positions[i];
							JointStateCmd.velocity[4] = msg->desired.velocities[i];
							//JointStateCmd.effort[4] = msg->effort[i];
					}
					else if(m_iNumDrives>=4 && msg->joint_names[i] ==  "fr_caster_r_wheel_joint")
					{
							JointStateCmd.position[6] = msg->desired.positions[i];
							JointStateCmd.velocity[6] = msg->desired.velocities[i];
							//JointStateCmd.effort[6] = msg->effort[i];
					}
					//STEERS
					else if(msg->joint_names[i] ==  "fl_caster_rotation_joint")
					{
							JointStateCmd.position[1] = msg->desired.positions[i];
							JointStateCmd.velocity[1] = msg->desired.velocities[i];
							//JointStateCmd.effort[1] = msg->effort[i];
					}
					else if(m_iNumDrives>=2 && msg->joint_names[i] ==  "bl_caster_rotation_joint")
					{
							JointStateCmd.position[3] = msg->desired.positions[i];
							JointStateCmd.velocity[3] = msg->desired.velocities[i];
							//JointStateCmd.effort[3] = msg->effort[i];
					}
					else if(m_iNumDrives>=3 && msg->joint_names[i] ==  "br_caster_rotation_joint")
					{
							JointStateCmd.position[5] = msg->desired.positions[i];
							JointStateCmd.velocity[5] = msg->desired.velocities[i];
							//JointStateCmd.effort[5] = msg->effort[i];
					}
					else if(m_iNumDrives>=4 && msg->joint_names[i] ==  "fr_caster_rotation_joint")
					{
							JointStateCmd.position[7] = msg->desired.positions[i];
							JointStateCmd.velocity[7] = msg->desired.velocities[i];
							//JointStateCmd.effort[7] = msg->effort[i];
					}
					else
					{
						ROS_ERROR("Unkown joint name %s", (msg->joint_names[i]).c_str());
					}
				}


				// check if velocities lie inside allowed boundaries
				for(int i = 0; i < m_iNumMotors; i++)
				{
#ifdef __SIM__
#else
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
#endif
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
				}

#ifdef __SIM__

#else
				if(m_bPubEffort) {
					m_CanCtrlPltf->requestMotorTorque();
				}
#endif
			}
		}

		// service callback functions
		// function will be called when a service is querried

		// Init Can-Configuration
		bool srvCallback_Init(std_srvs::Trigger::Request &req,
							  std_srvs::Trigger::Response &res )
		{
			ROS_DEBUG("Service callback init");
			if(m_bisInitialized == false)
			{
				m_bisInitialized = initDrives();
				//ROS_INFO("...initializing can-nodes...");
				//m_bisInitialized = m_CanCtrlPltf->initPltf();
				res.success = m_bisInitialized;
				if(m_bisInitialized)
				{
		   			ROS_INFO("base initialized");
				}
				else
				{
					res.message = "initialization of base failed";
				  	ROS_ERROR("Initializing base failed");
				}
			}
			else
			{
				ROS_WARN("...base already initialized...");
				res.success = true;
				res.message = "platform already initialized";
			}
			return true;
		}

		bool srvCallback_ElmoRecorderConfig(cob_base_drive_chain::ElmoRecorderConfig::Request &req,
							  cob_base_drive_chain::ElmoRecorderConfig::Response &res ){
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

		bool srvCallback_ElmoRecorderReadout(cob_base_drive_chain::ElmoRecorderReadout::Request &req,
							  cob_base_drive_chain::ElmoRecorderReadout::Response &res ){
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
		bool srvCallback_Recover(std_srvs::Trigger::Request &req,
									 std_srvs::Trigger::Response &res )
		{
			if(m_bisInitialized)
			{
				ROS_DEBUG("Service callback recover");
#ifdef __SIM__
				res.success = true;
#else
				res.success = m_CanCtrlPltf->resetPltf();
#endif
				if (res.success) {
		   			ROS_INFO("base resetted");
				} else {
					res.message = "reset of base failed";
					ROS_WARN("Resetting base failed");
				}
			}
			else
			{
				ROS_WARN("Base not yet initialized.");
				res.success = false;
				res.message = "Base not yet initialized.";
			}
			return true;
		}

		// shutdown Drivers and Can-Node
		bool srvCallback_Shutdown(std_srvs::Trigger::Request &req,
									 std_srvs::Trigger::Response &res )
		{
			ROS_DEBUG("Service callback shutdown");
#ifdef __SIM__
			res.success = true;
#else
			res.success = m_CanCtrlPltf->shutdownPltf();
#endif
			if (res.success)
	   			ROS_INFO("Drives shut down");
			else
	   			ROS_INFO("Shutdown of Drives FAILED");

			return true;
		}

		//publish JointStates cyclical instead of service callback
		bool publish_JointStates()
		{
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
			control_msgs::JointTrajectoryControllerState controller_state;

			// get time stamp for header
#ifdef __SIM__
			jointstate.header.stamp = m_gazeboStamp;
			controller_state.header.stamp = m_gazeboStamp;
#else
			jointstate.header.stamp = ros::Time::now();
			controller_state.header.stamp = jointstate.header.stamp;
#endif

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
				jointstate.name.resize(m_iNumMotors);

			}
			else
			{
				// as soon as drive chain is initialized
				// read Can-Buffer
#ifdef __SIM__

#else
				ROS_DEBUG("Read CAN-Buffer");
				m_CanCtrlPltf->evalCanBuffer();
				ROS_DEBUG("Successfully read CAN-Buffer");
#endif
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
				jointstate.name.resize(m_iNumMotors);

			}

			controller_state.joint_names = jointstate.name;
			controller_state.actual.positions = jointstate.position;
			controller_state.actual.velocities = jointstate.velocity;

			// publish jointstate message
#ifdef __SIM__
			//do not publish
#else
			topicPub_JointState.publish(jointstate);
#endif
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

		void publish_globalDiagnostics(const ros::TimerEvent& event)
		{
		  //publish global diagnostic messages
                  diagnostic_msgs::DiagnosticArray diagnostics_gl;
                  diagnostics_gl.header.stamp = ros::Time::now();
                  diagnostics_gl.status.resize(1);
                  // set data to diagnostics
#ifdef __SIM__
                  if (false)
#else
                  if(m_bisInitialized && m_CanCtrlPltf->isPltfError())
#endif

                  {
                    diagnostics_gl.status[0].level = 2;
                    diagnostics_gl.status[0].name = ros::this_node::getName();
                    diagnostics_gl.status[0].message = "Base not initialized or in error";
                  }
                  else
                  {
                    if (m_bisInitialized)
                    {
                      diagnostics_gl.status[0].level = 0;
                      diagnostics_gl.status[0].name = ros::this_node::getName();
                      diagnostics_gl.status[0].message = "base_drive_chain initialized and running";
                    }
                    else
                    {
                      diagnostics_gl.status[0].level = 1;
                      diagnostics_gl.status[0].name = ros::this_node::getName();
                      diagnostics_gl.status[0].message = "base_drive_chain not initialized";
                    }
                  }
                  // publish diagnostic message
                  topicPub_DiagnosticGlobal_.publish(diagnostics_gl);
		}

		// other function declarations
		bool initDrives();

#ifdef __SIM__
		void gazebo_joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg) {
			for (unsigned int i=0; i<msg->name.size(); i++) {
				//Drives
				if(msg->name[i] == "fl_caster_r_wheel_joint") {
					m_gazeboPos[0] = msg->position[i];
					m_gazeboVel[0] = msg->velocity[i];
					m_gazeboStamp = msg->header.stamp;
				}
				else if(msg->name[i] == "bl_caster_r_wheel_joint") {
					m_gazeboPos[2] = msg->position[i];
					m_gazeboVel[2] = msg->velocity[i];
				}
				else if(msg->name[i] == "br_caster_r_wheel_joint") {
					m_gazeboPos[4] = msg->position[i];
					m_gazeboVel[4] = msg->velocity[i];
				}
				else if(msg->name[i] == "fr_caster_r_wheel_joint") {
					m_gazeboPos[6] = msg->position[i];
					m_gazeboVel[6] = msg->velocity[i];
				}

				//Steers
				else if(msg->name[i] == "fl_caster_rotation_joint") {
					m_gazeboPos[1] = msg->position[i];
					m_gazeboVel[1] = msg->velocity[i];
				}
				else if(msg->name[i] == "bl_caster_rotation_joint") {
					m_gazeboPos[3] = msg->position[i];
					m_gazeboVel[3] = msg->velocity[i];
				}
				else if(msg->name[i] == "br_caster_rotation_joint") {
					m_gazeboPos[5] = msg->position[i];
					m_gazeboVel[5] = msg->velocity[i];
				}
				else if(msg->name[i] == "fr_caster_rotation_joint") {
					m_gazeboPos[7] = msg->position[i];
					m_gazeboVel[7] = msg->velocity[i];
				}
			}
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

#ifdef __SIM__
	// get Offset from Zero-Position of Steering
	if(m_iNumDrives >=1)
		m_Param.vdWheelNtrlPosRad[0] = 0.0f;
	if(m_iNumDrives >=2)
		m_Param.vdWheelNtrlPosRad[1] = 0.0f;
	if(m_iNumDrives >=3)
		m_Param.vdWheelNtrlPosRad[2] = 0.0f;
	if(m_iNumDrives >=4)
		m_Param.vdWheelNtrlPosRad[3] = 0.0f;
#else
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
#endif

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
