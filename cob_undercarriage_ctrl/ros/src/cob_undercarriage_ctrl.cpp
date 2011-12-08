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
 * ROS package name: cob_undercarriage_ctrl
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: April 2010:
 * ToDo: Adapt Interface to controller -> remove last variable (not used anymore)
 *       Rework Ctrl-class to work with SI-Units -> remove conversion
 *		 For easier association of joints use platform.urdf!!
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
#include <math.h>

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cob_relayboard/EmergencyStopState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>


// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_base_drive_chain/GetJointState.h>

// external includes
#include <cob_undercarriage_ctrl/UndercarriageCtrlGeom.h>
#include <cob_utilities/IniFile.h>
//#include <cob_utilities/MathSup.h>

//####################
//#### node class ####
class NodeClass
{
    //
    public:
	    // create a handle for this node, initialize node
	    ros::NodeHandle n;
                
        // topics to publish
        ros::Publisher topic_pub_joint_state_cmd_;	// cmd issued for single joints of undercarriage
		ros::Publisher topic_pub_controller_joint_command_;
        ros::Publisher topic_pub_odometry_;			// calculated (measured) velocity, rotation and pose (odometry-based) for the robot
        tf::TransformBroadcaster tf_broadcast_odometry_;	// according transformation for the tf broadcaster
        
	    // topics to subscribe, callback is called for new messages arriving
        ros::Subscriber topic_sub_CMD_pltf_twist_;	// issued command to be achieved by the platform
        ros::Subscriber topic_sub_EM_stop_state_;	// current emergency stop state (free, active, confirmed)
		ros::Subscriber topic_sub_drive_diagnostic_;// status of drive chain (initializing, error, normal)

		//subscribe to JointStates topic
		//ros::Subscriber topic_sub_joint_states_;
		ros::Subscriber topic_sub_joint_controller_states_;
        
        // service servers
        ros::ServiceServer srvServer_IsMoving;
            
	// diagnostic stuff
  	diagnostic_updater::Updater updater_;

        // service clients
        ros::ServiceClient srv_client_get_joint_state_;	// get current configuration of undercarriage

        // member variables
		UndercarriageCtrlGeom * ucar_ctrl_;	// instantiate undercarriage controller
		std::string sIniDirectory;
		bool is_initialized_bool_;			// flag wether node is already up and running
		bool is_moving_;					// flag wether base is moving or not
		int drive_chain_diagnostic_;		// flag whether base drive chain is operating normal 
		ros::Time last_time_;				// time Stamp for last odometry measurement
		double x_rob_m_, y_rob_m_, theta_rob_rad_; // accumulated motion of robot since startup
    	int iwatchdog_;
		
		int m_iNumJoints;
		
		diagnostic_msgs::DiagnosticStatus diagnostic_status_lookup_; // used to access defines for warning levels

        // Constructor
        NodeClass()
        {
			// initialization of variables
			is_initialized_bool_ = false;
			is_moving_ = false;
      iwatchdog_ = 0;
			last_time_ = ros::Time::now();
			x_rob_m_ = 0.0;
			y_rob_m_ = 0.0;
			theta_rob_rad_ = 0.0;
			// set status of drive chain to WARN by default
			drive_chain_diagnostic_ = diagnostic_status_lookup_.OK; //WARN; <- THATS FOR DEBUGGING ONLY!
			
			// Parameters are set within the launch file
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

			IniFile iniFile;
			iniFile.SetFileName(sIniDirectory + "Platform.ini", "PltfHardwareCoB3.h");
			iniFile.GetKeyInt("Config", "NumberOfMotors", &m_iNumJoints, true);
			
			ucar_ctrl_ = new UndercarriageCtrlGeom(sIniDirectory);
			
			
			// implementation of topics
            // published topics
			//topic_pub_joint_state_cmd_ = n.advertise<sensor_msgs::JointState>("joint_command", 1);
			topic_pub_controller_joint_command_ = n.advertise<pr2_controllers_msgs::JointTrajectoryControllerState> ("joint_command", 1);

			topic_pub_odometry_ = n.advertise<nav_msgs::Odometry>("odometry", 1);

            // subscribed topics
			topic_sub_CMD_pltf_twist_ = n.subscribe("command", 1, &NodeClass::topicCallbackTwistCmd, this);
            topic_sub_EM_stop_state_ = n.subscribe("/emergency_stop_state", 1, &NodeClass::topicCallbackEMStop, this);
            topic_sub_drive_diagnostic_ = n.subscribe("diagnostic", 1, &NodeClass::topicCallbackDiagnostic, this);

			
			
			//topic_sub_joint_states_ = n.subscribe("/joint_states", 1, &NodeClass::topicCallbackJointStates, this);
			topic_sub_joint_controller_states_ = n.subscribe("state", 1, &NodeClass::topicCallbackJointControllerStates, this);

			// diagnostics
			updater_.setHardwareID(ros::this_node::getName());
			updater_.add("initialization", this, &NodeClass::diag_init);

            // implementation of service servers
            srvServer_IsMoving = n.advertiseService("is_moving", &NodeClass::srvCallback_IsMoving, this);

			// implementation of service clients
            srv_client_get_joint_state_ = n.serviceClient<cob_base_drive_chain::GetJointState>("GetJointState");
        }
        
        // Destructor
        ~NodeClass() 
        {
        }

		void diag_init(diagnostic_updater::DiagnosticStatusWrapper &stat)
	  {
	    if(is_initialized_bool_)
	      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "");
	    else
	      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "");
	    stat.add("Initialized", is_initialized_bool_);
	  }


        // topic callback functions 
        // function will be called when a new message arrives on a topic

		// Listen for Pltf Cmds
		void topicCallbackTwistCmd(const geometry_msgs::Twist::ConstPtr& msg)
		{
			double vx_cmd_mms, vy_cmd_mms, w_cmd_rads;

			iwatchdog_ = 0;			

			// controller expects velocities in mm/s, ROS works with SI-Units -> convert
			// ToDo: rework Controller Class to work with SI-Units
			vx_cmd_mms = msg->linear.x*1000.0;
			vy_cmd_mms = msg->linear.y*1000.0;
			w_cmd_rads = msg->angular.z;

			// only process if controller is already initialized
			if (is_initialized_bool_)
            {
				ROS_DEBUG("received new velocity command [cmdVelX=%3.5f,cmdVelY=%3.5f,cmdVelTh=%3.5f]", 
                     msg->linear.x, msg->linear.y, msg->angular.z);

				// Set desired value for Plattform Velocity to UndercarriageCtrl (setpoint setting)
				ucar_ctrl_->SetDesiredPltfVelocity(vx_cmd_mms, vy_cmd_mms, w_cmd_rads, 0.0);
				// ToDo: last value (0.0) is not used anymore --> remove from interface
			}
			else
			{	
				// Set desired value for Plattform Velocity to zero (setpoint setting)
				ucar_ctrl_->SetDesiredPltfVelocity( 0.0, 0.0, 0.0, 0.0);
				// ToDo: last value (0.0) is not used anymore --> remove from interface
				ROS_DEBUG("Forced platform-velocity cmds to zero");
			}
		}

		// Listen for Emergency Stop
		void topicCallbackEMStop(const cob_relayboard::EmergencyStopState::ConstPtr& msg)
		{
			int EM_state;
			EM_state = msg->emergency_state;

			if (EM_state == msg->EMFREE)
			{
				// Reset EM flag in Ctrlr
				if (is_initialized_bool_) 
				{
					ucar_ctrl_->setEMStopActive(false);
					ROS_DEBUG("Undercarriage Controller EM-Stop released");
					// reset only done, when system initialized
					// -> allows to stop ctrlr during init, reset and shutdown
				}
			}
			else
			{
            	ROS_DEBUG("Undercarriage Controller stopped due to EM-Stop");

				// Set desired value for Plattform Velocity to zero (setpoint setting)
				ucar_ctrl_->SetDesiredPltfVelocity( 0.0, 0.0, 0.0, 0.0);
				// ToDo: last value (0.0) is not used anymore --> remove from interface
				ROS_DEBUG("Forced platform-velocity cmds to zero");
		
				// Set EM flag and stop Ctrlr
				ucar_ctrl_->setEMStopActive(true);
			}
		}

		// Listens for status of underlying hardware (base drive chain)
		void topicCallbackDiagnostic(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg)
		{
			pr2_controllers_msgs::JointTrajectoryControllerState joint_state_cmd;

			// prepare joint_cmds for heartbeat (compose header)
			joint_state_cmd.header.stamp = ros::Time::now();
			//joint_state_cmd.header.frame_id = frame_id; //Where to get this id from?
			// ToDo: configure over Config-File (number of motors) and Msg
			// assign right size to JointState data containers
			//joint_state_cmd.set_name_size(m_iNumMotors);
			joint_state_cmd.desired.positions.resize(m_iNumJoints);
			joint_state_cmd.desired.velocities.resize(m_iNumJoints);            
			//joint_state_cmd.desired.effort.resize(m_iNumJoints);
			joint_state_cmd.joint_names.push_back("fl_caster_r_wheel_joint");
			joint_state_cmd.joint_names.push_back("fl_caster_rotation_joint");
			joint_state_cmd.joint_names.push_back("bl_caster_r_wheel_joint");
			joint_state_cmd.joint_names.push_back("bl_caster_rotation_joint");
			joint_state_cmd.joint_names.push_back("br_caster_r_wheel_joint");
			joint_state_cmd.joint_names.push_back("br_caster_rotation_joint");
			joint_state_cmd.joint_names.push_back("fr_caster_r_wheel_joint");
			joint_state_cmd.joint_names.push_back("fr_caster_rotation_joint");
			// compose jointcmds
			for(int i=0; i<m_iNumJoints; i++)
			{					
				joint_state_cmd.desired.positions[i] = 0.0;
				joint_state_cmd.desired.velocities[i] = 0.0;
				//joint_state_cmd.desired.effort[i] = 0.0;
			}
			
			// set status of underlying drive chain to member variable 
			drive_chain_diagnostic_ = msg->level;

			// if controller is already started up ...
			if (is_initialized_bool_)
			{
				// ... but underlying drive chain is not yet operating normal
				if (drive_chain_diagnostic_ != diagnostic_status_lookup_.OK)
				{
					// halt controller
            		ROS_DEBUG("drive chain not availlable: halt Controller");

					// Set EM flag to Ctrlr (resets internal states)
					ucar_ctrl_->setEMStopActive(true);

					// Set desired value for Plattform Velocity to zero (setpoint setting)
					ucar_ctrl_->SetDesiredPltfVelocity( 0.0, 0.0, 0.0, 0.0);
					// ToDo: last value (0.0) is not used anymore --> remove from interface
					ROS_DEBUG("Forced platform-velocity cmds to zero");
					
					// if is not Initializing
					if (drive_chain_diagnostic_ != diagnostic_status_lookup_.WARN)
					{
						// publish zero-vel. jointcmds to avoid Watchdogs stopping ctrlr
						topic_pub_controller_joint_command_.publish(joint_state_cmd);
					}
				}
			}
			// ... while controller is not initialized send heartbeats to keep motors alive
			else
			{
				// ... as soon as base drive chain is initialized
				if(drive_chain_diagnostic_ != diagnostic_status_lookup_.WARN)
				{
					// publish zero-vel. jointcmds to avoid Watchdogs stopping ctrlr
					topic_pub_controller_joint_command_.publish(joint_state_cmd);
				}
			}
		}



        // service callback functions
        // function will be called when a service is querried
        bool srvCallback_IsMoving(cob_srvs::Trigger::Request &req,
							  	cob_srvs::Trigger::Response &res )
		{
			ROS_DEBUG("Service Callback is_moving");
			res.success.data = is_moving_;
			return true;
		}

/*		// Init Controller Configuration
        bool srvCallbackInit(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res )
        {
            if(is_initialized_bool_ == false)
            {
				is_initialized_bool_ = InitCtrl();
				// intializes some configuration variabes
				res.success.data = is_initialized_bool_;
           	    
				if (is_initialized_bool_)
				{
					ROS_INFO("Undercarriage-Ctrl initialized");
					// reset Time
					last_time_ = ros::Time::now();
				}
				else
				{
					ROS_INFO("Undercarriage-Ctrl initialization failed");
                	res.error_message.data = "initialization of undercarriage controller failed";
				}
            }
            else
            {
                ROS_ERROR("... undercarriage controller already initialized...");
                res.success.data = false;
                res.error_message.data = "undercarriage controller already initialized";
            }            
            return true;
        }
*/
	
/*		// reset Controller Configuration
        bool srvCallbackReset(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res )
        {
			bool ctrlr_reset;

			if (is_initialized_bool_)
			{

				// flag that Controller is not initialized
				is_initialized_bool_ = false;

				// first of all stop controller (similar to EMStop)
				// Set desired value for Plattform Velocity to zero (setpoint setting)
				ucar_ctrl_->SetDesiredPltfVelocity( 0.0, 0.0, 0.0, 0.0);
				// ToDo: last value (0.0) is not used anymore --> remove from interface
				ROS_DEBUG("Forced platform-velocity cmds to zero");
				// Set EM flag and stop Ctrlr
				ucar_ctrl_->setEMStopActive(true);

				// now re-init controller configuration
				ctrlr_reset = InitCtrl();

				if (ctrlr_reset)
				{
					// restart controller
					// Set desired value for Plattform Velocity to zero (setpoint setting)
					ucar_ctrl_->SetDesiredPltfVelocity( 0.0, 0.0, 0.0, 0.0);
					// ToDo: last value (0.0) is not used anymore --> remove from interface
					ROS_DEBUG("Forced platform-velocity cmds to zero");
					// Set EM flag and stop Ctrlr
					ucar_ctrl_->setEMStopActive(false);

					// reset Time
					last_time_ = ros::Time::now();
					// and reset odometry
					x_rob_m_ = 0.0;
					y_rob_m_ = 0.0;
					theta_rob_rad_ = 0.0;

					// flag that controller is initialized
					is_initialized_bool_ = true;

					// set answer for service request
	       	    	ROS_INFO("Undercarriage Controller resetted");
					res.success.data = true;
				}
				else
				{
					// set answer for service request
	       	    	ROS_INFO("Re-Init after Reset of Undercarriage Controller failed");
					res.success.data = false;
                	res.error_message.data = "reinit after reset of undercarriage controller failed";
				}
			}
			else
			{
				// Reset not possible, because Controller not yet set up
                ROS_ERROR("... undercarriage controller not yet initialized, reset not possible ...");
				// set answer for service request
                res.success.data = false;
                res.error_message.data = "undercarriage controller not yet initialized";
			}

		    return true;
        }
*/
		
/*		// shutdown Controller
        bool srvCallbackShutdown(cob_srvs::Trigger::Request &req,
                                     cob_srvs::Trigger::Response &res )
        {
			if (is_initialized_bool_)
			{
				// stop controller
				// Set desired value for Plattform Velocity to zero (setpoint setting)
				ucar_ctrl_->SetDesiredPltfVelocity( 0.0, 0.0, 0.0, 0.0);
				ROS_DEBUG("Forced platform-velocity cmds to zero");

				// flag that controller is not running anymore
				is_initialized_bool_ = false;
	
				res.success.data = true;
			}
			else
			{
				// shutdown not possible, because pltf not running
                ROS_ERROR("...platform not initialized...");
                res.success.data = false;
                res.error_message.data = "platform already or still down";
			}
	    	return true;
        }
*/

		void topicCallbackJointControllerStates(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg) {
			int num_joints;
			int iter_k, iter_j;
			std::vector<double> drive_joint_ang_rad, drive_joint_vel_rads, drive_joint_effort_NM;
			std::vector<double> steer_joint_ang_rad, steer_joint_vel_rads, steer_joint_effort_NM;
			cob_base_drive_chain::GetJointState srv_get_joint;
	
			// copy configuration into vector classes
			num_joints = msg->joint_names.size();
			// drive joints
			drive_joint_ang_rad.assign(m_iNumJoints, 0.0);
			drive_joint_vel_rads.assign(m_iNumJoints, 0.0);
			drive_joint_effort_NM.assign(m_iNumJoints, 0.0);
			// steer joints
			steer_joint_ang_rad.assign(m_iNumJoints, 0.0);
			steer_joint_vel_rads.assign(m_iNumJoints, 0.0);
			steer_joint_effort_NM.assign(m_iNumJoints, 0.0);

			// init iterators
			iter_k = 0;
			iter_j = 0;

			for(int i = 0; i < num_joints; i++)
			{
				// associate inputs to according steer and drive joints
				// ToDo: specify this globally (Prms-File or config-File or via msg-def.)	
				if(msg->joint_names[i] ==  "fl_caster_r_wheel_joint")
				{
						drive_joint_ang_rad[0] = msg->actual.positions[i]; 
						drive_joint_vel_rads[0] = msg->actual.velocities[i];
						//drive_joint_effort_NM[0] = msg->effort[i];
				}
				if(msg->joint_names[i] ==  "bl_caster_r_wheel_joint")
				{
						drive_joint_ang_rad[1] = msg->actual.positions[i]; 
						drive_joint_vel_rads[1] = msg->actual.velocities[i];
						//drive_joint_effort_NM[1] = msg->effort[i];
				}
				if(msg->joint_names[i] ==  "br_caster_r_wheel_joint")
				{
						drive_joint_ang_rad[2] = msg->actual.positions[i]; 
						drive_joint_vel_rads[2] = msg->actual.velocities[i];
						//drive_joint_effort_NM[2] = msg->effort[i];
				}
				if(msg->joint_names[i] ==  "fr_caster_r_wheel_joint")
				{
						drive_joint_ang_rad[3] = msg->actual.positions[i]; 
						drive_joint_vel_rads[3] = msg->actual.velocities[i];
						//drive_joint_effort_NM[3] = msg->effort[i];
				}
				if(msg->joint_names[i] ==  "fl_caster_rotation_joint")
				{
						steer_joint_ang_rad[0] = msg->actual.positions[i]; 
						steer_joint_vel_rads[0] = msg->actual.velocities[i];
						//steer_joint_effort_NM[0] = msg->effort[i];
				}
				if(msg->joint_names[i] ==  "bl_caster_rotation_joint")
				{ 
						steer_joint_ang_rad[1] = msg->actual.positions[i]; 
						steer_joint_vel_rads[1] = msg->actual.velocities[i];
						//steer_joint_effort_NM[1] = msg->effort[i];
				}
				if(msg->joint_names[i] ==  "br_caster_rotation_joint")
				{
						steer_joint_ang_rad[2] = msg->actual.positions[i]; 
						steer_joint_vel_rads[2] = msg->actual.velocities[i];
						//steer_joint_effort_NM[2] = msg->effort[i];
				}
				if(msg->joint_names[i] ==  "fr_caster_rotation_joint")
				{
						steer_joint_ang_rad[3] = msg->actual.positions[i]; 
						steer_joint_vel_rads[3] = msg->actual.velocities[i];
						//steer_joint_effort_NM[3] = msg->effort[i];
				}
				
			}

			// Set measured Wheel Velocities and Angles to Controler Class (implements inverse kinematic)
			ucar_ctrl_->SetActualWheelValues(drive_joint_vel_rads, steer_joint_vel_rads,
									drive_joint_ang_rad, steer_joint_ang_rad);

			
			
			/*pr2_controllers_msgs::JointTrajectoryControllerState controller_state_msg;

			controller_state_msg.header.stamp = msg->header.stamp;
			controller_state_msg.actual.positions.resize(m_iNumJoints);
			controller_state_msg.actual.velocities.resize(m_iNumJoints);            
			controller_state_msg.actual.accelerations.resize(m_iNumJoints);
			controller_state_msg.joint_names.push_back("fl_caster_r_wheel_joint");
			controller_state_msg.joint_names.push_back("fl_caster_rotation_joint");
			controller_state_msg.joint_names.push_back("bl_caster_r_wheel_joint");
			controller_state_msg.joint_names.push_back("bl_caster_rotation_joint");
			controller_state_msg.joint_names.push_back("br_caster_r_wheel_joint");
			controller_state_msg.joint_names.push_back("br_caster_rotation_joint");
			controller_state_msg.joint_names.push_back("fr_caster_r_wheel_joint");
			controller_state_msg.joint_names.push_back("fr_caster_rotation_joint");
			controller_state_msg.actual.positions = msg->position;
			controller_state_msg.actual.velocities = msg->velocity;
			controller_state_msg.actual.accelerations = msg->effort;
	
			topic_pub_controller_state_.publish(controller_state_msg);*/		
		}


		/*
		void topicCallbackJointStates(const sensor_msgs::JointState::ConstPtr& msg) {
			int num_joints;
			int iter_k, iter_j;
			std::vector<double> drive_joint_ang_rad, drive_joint_vel_rads, drive_joint_effort_NM;
			std::vector<double> steer_joint_ang_rad, steer_joint_vel_rads, steer_joint_effort_NM;
			cob_base_drive_chain::GetJointState srv_get_joint;
	
			// copy configuration into vector classes
			num_joints = msg->position.size();
			// drive joints
			drive_joint_ang_rad.assign(m_iNumJoints, 0.0);
			drive_joint_vel_rads.assign(m_iNumJoints, 0.0);
			drive_joint_effort_NM.assign(m_iNumJoints, 0.0);
			// steer joints
			steer_joint_ang_rad.assign(m_iNumJoints, 0.0);
			steer_joint_vel_rads.assign(m_iNumJoints, 0.0);
			steer_joint_effort_NM.assign(m_iNumJoints, 0.0);

			// init iterators
			iter_k = 0;
			iter_j = 0;

			for(int i = 0; i < num_joints; i++)
			{
				// associate inputs to according steer and drive joints
				// ToDo: specify this globally (Prms-File or config-File or via msg-def.)	
				if(msg->name[i] ==  "fl_caster_r_wheel_joint")
				{
						drive_joint_ang_rad[0] = msg->position[i]; 
						drive_joint_vel_rads[0] = msg->velocity[i];
						drive_joint_effort_NM[0] = msg->effort[i];
				}
				if(msg->name[i] ==  "bl_caster_r_wheel_joint")
				{
						drive_joint_ang_rad[1] = msg->position[i]; 
						drive_joint_vel_rads[1] = msg->velocity[i];
						drive_joint_effort_NM[1] = msg->effort[i];
				}
				if(msg->name[i] ==  "br_caster_r_wheel_joint")
				{
						drive_joint_ang_rad[2] = msg->position[i]; 
						drive_joint_vel_rads[2] = msg->velocity[i];
						drive_joint_effort_NM[2] = msg->effort[i];
				}
				if(msg->name[i] ==  "fr_caster_r_wheel_joint")
				{
						drive_joint_ang_rad[3] = msg->position[i]; 
						drive_joint_vel_rads[3] = msg->velocity[i];
						drive_joint_effort_NM[3] = msg->effort[i];
				}
				if(msg->name[i] ==  "fl_caster_rotation_joint")
				{
						steer_joint_ang_rad[0] = msg->position[i]; 
						steer_joint_vel_rads[0] = msg->velocity[i];
						steer_joint_effort_NM[0] = msg->effort[i];
				}
				if(msg->name[i] ==  "bl_caster_rotation_joint")
				{ 
						steer_joint_ang_rad[1] = msg->position[i]; 
						steer_joint_vel_rads[1] = msg->velocity[i];
						steer_joint_effort_NM[1] = msg->effort[i];
				}
				if(msg->name[i] ==  "br_caster_rotation_joint")
				{
						steer_joint_ang_rad[2] = msg->position[i]; 
						steer_joint_vel_rads[2] = msg->velocity[i];
						steer_joint_effort_NM[2] = msg->effort[i];
				}
				if(msg->name[i] ==  "fr_caster_rotation_joint")
				{
						steer_joint_ang_rad[3] = msg->position[i]; 
						steer_joint_vel_rads[3] = msg->velocity[i];
						steer_joint_effort_NM[3] = msg->effort[i];
				}
				
			}

			// Set measured Wheel Velocities and Angles to Controler Class (implements inverse kinematic)
			ucar_ctrl_->SetActualWheelValues(drive_joint_vel_rads, steer_joint_vel_rads,
									drive_joint_ang_rad, steer_joint_ang_rad);

			pr2_controllers_msgs::JointTrajectoryControllerState controller_state_msg;

			controller_state_msg.header.stamp = msg->header.stamp;
			controller_state_msg.actual.positions.resize(m_iNumJoints);
			controller_state_msg.actual.velocities.resize(m_iNumJoints);            
			controller_state_msg.actual.accelerations.resize(m_iNumJoints);
			controller_state_msg.joint_names.push_back("fl_caster_r_wheel_joint");
			controller_state_msg.joint_names.push_back("fl_caster_rotation_joint");
			controller_state_msg.joint_names.push_back("bl_caster_r_wheel_joint");
			controller_state_msg.joint_names.push_back("bl_caster_rotation_joint");
			controller_state_msg.joint_names.push_back("br_caster_r_wheel_joint");
			controller_state_msg.joint_names.push_back("br_caster_rotation_joint");
			controller_state_msg.joint_names.push_back("fr_caster_r_wheel_joint");
			controller_state_msg.joint_names.push_back("fr_caster_rotation_joint");
			controller_state_msg.actual.positions = msg->position;
			controller_state_msg.actual.velocities = msg->velocity;
			controller_state_msg.actual.accelerations = msg->effort;
	
			topic_pub_controller_state_.publish(controller_state_msg);

		}
		
		*/

       
        // other function declarations
		// Initializes controller
        bool InitCtrl();
		// perform one control step, calculate inverse kinematics and publish updated joint cmd's (if no EMStop occurred)
		void CalcCtrlStep();
		// acquires the current undercarriage configuration from base_drive_chain
		//void GetJointState(); // TODO: not used any more, can be deleted
		// calculates odometry from current measurement values and publishes it via an odometry topic and the tf broadcaster
		void UpdateOdometry();
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "undercarriage_ctrl");
    
	// construct nodeClass
    NodeClass nodeClass;
	
	// automatically do initializing of controller, because it's not directly depending any hardware components
	nodeClass.is_initialized_bool_ = nodeClass.InitCtrl();
	if( nodeClass.is_initialized_bool_ ) {
		nodeClass.last_time_ = ros::Time::now();
		ROS_INFO("Undercarriage control successfully initialized.");
	} else
		ROS_WARN("Undercarriage control initialization failed! Try manually.");
	
	// specify looprate of control-cycle
 	ros::Rate loop_rate(50); // Hz 
    
    while(nodeClass.n.ok())
    {
		// process Callbacks (get new pltf-cmd's and check for EMStop)
        ros::spinOnce();

		// request Update of Undercarriage Configuration
		// EXPERIMENTAL: listen to JointStates via topic
		// nodeClass.GetJointState();
		
		// perform one control step, calculate inverse kinematics
		// and publish updated joint cmd's (if no EMStop occurred)
		nodeClass.CalcCtrlStep();

		// calculate forward kinematics and update Odometry
		nodeClass.UpdateOdometry();

		// -> let node sleep for the rest of the cycle
        loop_rate.sleep();
    }

    return 0;
}

//##################################
//#### function implementations ####
bool NodeClass::InitCtrl()
{
    ROS_INFO("Initializing Undercarriage Controller");

    // ToDo:
	// 1st step: pass the path to the Inifile via the Parameterserver
	// 2nd step: replace the Inifiles by ROS configuration files
    
	// create Inifile class and set target inifile (from which data shall be read)
	//IniFile iniFile;
	// Parameters are set within the launch file
	//n.param<std::string>("base_drive_chain_node/IniDirectory", sIniDirectory, "Platform/IniFiles/");
	//ROS_INFO("IniDirectory loaded from Parameter-Server is: %s", sIniDirectory.c_str());
	//iniFile.SetFileName(sIniDirectory + "Platform.ini", "PltfHardwareCoB3.h");

	// Init Controller Class
	ucar_ctrl_->InitUndercarriageCtrl();
	ROS_INFO("Initializing Undercarriage Controller done");

    return true;
}

// perform one control step, calculate inverse kinematics and publish updated joint cmd's (if no EMStop occurred)
void NodeClass::CalcCtrlStep()
{
	double vx_cmd_ms, vy_cmd_ms, w_cmd_rads, dummy;
	std::vector<double> drive_jointvel_cmds_rads, steer_jointvel_cmds_rads, steer_jointang_cmds_rad;
	pr2_controllers_msgs::JointTrajectoryControllerState joint_state_cmd;
	int j, k;
  iwatchdog_ += 1;	

	// if controller is initialized and underlying hardware is operating normal
	if (is_initialized_bool_) //&& (drive_chain_diagnostic_ != diagnostic_status_lookup_.OK))
	{
		// as soon as (but only as soon as) platform drive chain is initialized start to send velocity commands
		// Note: topicCallbackDiagnostic checks whether drives are operating nominal.
		//       -> if warning or errors are issued target velocity is set to zero

		// perform one control step,
		// get the resulting cmd's for the wheel velocities and -angles from the controller class
		// and output the achievable pltf velocity-cmds (if velocity limits where exceeded)
		ucar_ctrl_->GetNewCtrlStateSteerDriveSetValues(drive_jointvel_cmds_rads,  steer_jointvel_cmds_rads, steer_jointang_cmds_rad, vx_cmd_ms, vy_cmd_ms, w_cmd_rads, dummy);
		// ToDo: adapt interface of controller class --> remove last values (not used anymore)

		// if drives not operating nominal -> force commands to zero
		if(drive_chain_diagnostic_ != diagnostic_status_lookup_.OK)
		{
			steer_jointang_cmds_rad.assign(m_iNumJoints, 0.0);
			steer_jointvel_cmds_rads.assign(m_iNumJoints, 0.0);
			
		}

		// convert variables to SI-Units
		vx_cmd_ms = vx_cmd_ms/1000.0;
		vy_cmd_ms = vy_cmd_ms/1000.0;

		// compose jointcmds
		// compose header
		joint_state_cmd.header.stamp = ros::Time::now();
		//joint_state_cmd.header.frame_id = frame_id; //Where to get this id from?
		// ToDo: configure over Config-File (number of motors) and Msg
		// assign right size to JointState data containers
		//joint_state_cmd.set_name_size(m_iNumMotors);
		joint_state_cmd.desired.positions.resize(m_iNumJoints);
		joint_state_cmd.desired.velocities.resize(m_iNumJoints);            
		//joint_state_cmd.effort.resize(m_iNumJoints);
		joint_state_cmd.joint_names.push_back("fl_caster_r_wheel_joint");
		joint_state_cmd.joint_names.push_back("fl_caster_rotation_joint");
		joint_state_cmd.joint_names.push_back("bl_caster_r_wheel_joint");
		joint_state_cmd.joint_names.push_back("bl_caster_rotation_joint");
		joint_state_cmd.joint_names.push_back("br_caster_r_wheel_joint");
		joint_state_cmd.joint_names.push_back("br_caster_rotation_joint");
		joint_state_cmd.joint_names.push_back("fr_caster_r_wheel_joint");
		joint_state_cmd.joint_names.push_back("fr_caster_rotation_joint");

		// compose data body
		j = 0;
		k = 0;
		for(int i = 0; i<m_iNumJoints; i++)
		{
      if(iwatchdog_ < 50)
      {
			  // for steering motors
			  if( i == 1 || i == 3 || i == 5 || i == 7) // ToDo: specify this via the Msg
			  {
				  joint_state_cmd.desired.positions[i] = steer_jointang_cmds_rad[j];
				  joint_state_cmd.desired.velocities[i] = steer_jointvel_cmds_rads[j];
				  //joint_state_cmd.effort[i] = 0.0;
				  j = j + 1;
			  }
			  else
			  {
				  joint_state_cmd.desired.positions[i] = 0.0;
				  joint_state_cmd.desired.velocities[i] = drive_jointvel_cmds_rads[k];
				  //joint_state_cmd.effort[i] = 0.0;
				  k = k + 1;
			  }
      }
      else
      {
          joint_state_cmd.desired.positions[i] = 0.0;
		  joint_state_cmd.desired.velocities[i] = 0.0;
		  //joint_state_cmd.effort[i] = 0.0;
      }
		}

		// publish jointcmds
		topic_pub_controller_joint_command_.publish(joint_state_cmd);
	}

}

/*
// acquires the current undercarriage configuration from base_drive_chain
void NodeClass::GetJointState()
{
	int num_joints;
	int iter_k, iter_j;
	std::vector<double> drive_joint_ang_rad, drive_joint_vel_rads, drive_joint_effort_NM;
	std::vector<double> steer_joint_ang_rad, steer_joint_vel_rads, steer_joint_effort_NM;
	cob_base_drive_chain::GetJointState srv_get_joint;
	
	// request update for pltf config --> call GetJointstate service
	srv_client_get_joint_state_.call(srv_get_joint);

	// copy configuration into vector classes
	num_joints = srv_get_joint.response.jointstate.position.size();
	// drive joints
	drive_joint_ang_rad.assign(num_joints, 0.0);
	drive_joint_vel_rads.assign(num_joints, 0.0);
	drive_joint_effort_NM.assign(num_joints, 0.0);
	// steer joints
	steer_joint_ang_rad.assign(num_joints, 0.0);
	steer_joint_vel_rads.assign(num_joints, 0.0);
	steer_joint_effort_NM.assign(num_joints, 0.0);

	// init iterators
	iter_k = 0;
	iter_j = 0;

	for(int i = 0; i < num_joints; i++)
	{
		// associate inputs to according steer and drive joints
		// ToDo: specify this globally (Prms-File or config-File or via msg-def.)
		// ToDo: use joint names instead of magic integers
		if( i == 1 || i == 3 || i == 5 || i == 7)
		{
			steer_joint_ang_rad[iter_k] = srv_get_joint.response.jointstate.position[i];
			steer_joint_vel_rads[iter_k] = srv_get_joint.response.jointstate.velocity[i];
			steer_joint_effort_NM[iter_k] = srv_get_joint.response.jointstate.effort[i];
			iter_k = iter_k + 1;
		}
		else
		{
			drive_joint_ang_rad[iter_j] = srv_get_joint.response.jointstate.position[i];
			drive_joint_vel_rads[iter_j] = srv_get_joint.response.jointstate.velocity[i];
			drive_joint_effort_NM[iter_j] = srv_get_joint.response.jointstate.effort[i];
			iter_j = iter_j + 1;
		}
	}

	// Set measured Wheel Velocities and Angles to Controler Class (implements inverse kinematic)
	ucar_ctrl_->SetActualWheelValues(drive_joint_vel_rads, steer_joint_vel_rads,
							drive_joint_ang_rad, steer_joint_ang_rad);

}
*/

// calculates odometry from current measurement values
// and publishes it via an odometry topic and the tf broadcaster
void NodeClass::UpdateOdometry()
{
	double vel_x_rob_ms, vel_y_rob_ms, rot_rob_rads, delta_x_rob_m, delta_y_rob_m, delta_theta_rob_rad;
	double dummy1, dummy2;
	double dt;
	ros::Time current_time;

	// if drive chain already initialized process joint data
	//if (drive_chain_diagnostic_ != diagnostic_status_lookup_.OK)
	if (is_initialized_bool_)
	{
		// Get resulting Pltf Velocities from Ctrl-Class (result of forward kinematics)
		// !Careful! Controller internally calculates with mm instead of m
		// ToDo: change internal calculation to SI-Units
		// ToDo: last values are not used anymore --> remove from interface
		ucar_ctrl_->GetActualPltfVelocity(delta_x_rob_m, delta_y_rob_m, delta_theta_rob_rad, dummy1,
									vel_x_rob_ms, vel_y_rob_ms, rot_rob_rads, dummy2);
		
		// convert variables to SI-Units
		vel_x_rob_ms = vel_x_rob_ms/1000.0;
		vel_y_rob_ms = vel_y_rob_ms/1000.0;
		delta_x_rob_m = delta_x_rob_m/1000.0;
		delta_y_rob_m = delta_y_rob_m/1000.0;
		
		ROS_DEBUG("Odmonetry delta is: x=%f, y=%f, th=%f", delta_x_rob_m, delta_y_rob_m, rot_rob_rads);
	}
	else
	{
		// otherwise set data (velocity and pose-delta) to zero
		vel_x_rob_ms = 0.0;
		vel_y_rob_ms = 0.0;
		delta_x_rob_m = 0.0;
		delta_y_rob_m = 0.0;
	}

	// calc odometry (from startup)
	// get time since last odometry-measurement
	current_time = ros::Time::now();
	dt = current_time.toSec() - last_time_.toSec();
	last_time_ = current_time;
	// calculation from ROS odom publisher tutorial http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Odom
	x_rob_m_ = x_rob_m_ + (vel_x_rob_ms * cos(theta_rob_rad_) - vel_y_rob_ms * sin(theta_rob_rad_)) * dt;
	y_rob_m_ = y_rob_m_ + (vel_x_rob_ms * sin(theta_rob_rad_) + vel_y_rob_ms * cos(theta_rob_rad_)) * dt;
	theta_rob_rad_ = theta_rob_rad_ + rot_rob_rads * dt;

	// format data for compatibility with tf-package and standard odometry msg
	// generate quaternion for rotation
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_rob_rad_);

	// compose and publish transform for tf package
	geometry_msgs::TransformStamped odom_tf;
	// compose header
	odom_tf.header.stamp = current_time;
	odom_tf.header.frame_id = "/wheelodom";
	odom_tf.child_frame_id = "/base_footprint";
	// compose data container
	odom_tf.transform.translation.x = x_rob_m_;
	odom_tf.transform.translation.y = y_rob_m_;
	odom_tf.transform.translation.z = 0.0;
	odom_tf.transform.rotation = odom_quat;

    // compose and publish odometry message as topic
    nav_msgs::Odometry odom_top;
	// compose header
    odom_top.header.stamp = current_time;
    odom_top.header.frame_id = "/wheelodom";
    odom_top.child_frame_id = "/base_footprint";
    // compose pose of robot
    odom_top.pose.pose.position.x = x_rob_m_;
    odom_top.pose.pose.position.y = y_rob_m_;
    odom_top.pose.pose.position.z = 0.0;
    odom_top.pose.pose.orientation = odom_quat;
    for(int i = 0; i < 6; i++)
      odom_top.pose.covariance[i*6+i] = 0.1;

    // compose twist of robot
    odom_top.twist.twist.linear.x = vel_x_rob_ms;
    odom_top.twist.twist.linear.y = vel_y_rob_ms;
    odom_top.twist.twist.linear.z = 0.0;
    odom_top.twist.twist.angular.x = 0.0;
    odom_top.twist.twist.angular.y = 0.0;
    odom_top.twist.twist.angular.z = rot_rob_rads;
    for(int i = 0; i < 6; i++)
      odom_top.twist.covariance[6*i+i] = 0.1;
	// publish data
	// publish the transform
	//tf_broadcast_odometry_.sendTransform(odom_tf);
	
	if (fabs(vel_x_rob_ms) > 0.005 or fabs(vel_y_rob_ms) > 0.005 or fabs(rot_rob_rads) > 0.005)
	{
		is_moving_ = true;
	}
	else
	{
		is_moving_ = false;
	}
	
	// publish odometry msg
	topic_pub_odometry_.publish(odom_top);
}
