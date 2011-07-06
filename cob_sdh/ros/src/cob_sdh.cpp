/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2010 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_driver
 * \note
 *   ROS package name: cob_sdh
 *
 * \author
 *   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * \author
 *   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 *
 * \date Date of creation: Jan 2010
 *
 * \brief
 *   Implementation of ROS node for sdh.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
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
#include <unistd.h>

// ROS includes
#include <ros/ros.h>
#include <urdf/model.h>
#include <actionlib/server/simple_action_server.h>

// ROS message includes
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <cob_sdh/TactileSensor.h>
#include <cob_sdh/TactileMatrix.h>

// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>

// ROS diagnostic msgs
#include <diagnostic_updater/diagnostic_updater.h>

// external includes
#include <cob_sdh/sdh.h>
#include <cob_sdh/dsa.h>

/*!
* \brief Implementation of ROS node for sdh.
*
* Offers actionlib and direct command interface.
*/
class SdhNode
{
	public:
		/// create a handle for this node, initialize node
		ros::NodeHandle nh_;
	private:
		// declaration of topics to publish
		ros::Publisher topicPub_JointState_;
		ros::Publisher topicPub_ControllerState_;
		ros::Publisher topicPub_TactileSensor_;

		// service servers
		ros::ServiceServer srvServer_Init_;
		ros::ServiceServer srvServer_Stop_;
		ros::ServiceServer srvServer_Recover_;
		ros::ServiceServer srvServer_SetOperationMode_;

		// actionlib server
		actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> as_;
		std::string action_name_;
		//cob_actions::JointCommandFeedback feedback_;
		//cob_actions::JointCommandResult result_;

		// service clients
		//--

		//diagnostics
		diagnostic_updater::Updater updater_;

		// other variables
		SDH::cSDH *sdh_;
		SDH::cDSA *dsa_;
		std::vector<SDH::cSDH::eAxisState> state_;

		std::string sdhdevicetype_;
		std::string sdhdevicestring_;
		int sdhdevicenum_;
		std::string dsadevicestring_;
		int dsadevicenum_;
		int baudrate_, id_read_, id_write_;
		double timeout_;

		bool isInitialized_;
		bool isDSAInitialized_;
		int DOF_;
		double pi_;
		
		trajectory_msgs::JointTrajectory traj_;
		
		std::vector<std::string> JointNames_;
		std::vector<int> axes_;
		std::vector<double> targetAngles_; // in degrees
		bool hasNewGoal_;
		
	public:
		/*!
		* \brief Constructor for SdhNode class
		*
		* \param name Name for the actionlib server
		*/
		SdhNode(std::string name):
			as_(nh_, name, boost::bind(&SdhNode::executeCB, this, _1),true),
			action_name_(name)
		{
			pi_ = 3.1415926;
			// diagnostics
			updater_.setHardwareID(ros::this_node::getName());
			updater_.add("initialization", this, &SdhNode::diag_init);

		}

		/*!
		* \brief Destructor for SdhNode class
		*/
		~SdhNode() 
		{
			if(isDSAInitialized_)
				dsa_->Close();
			if(isInitialized_)
				sdh_->Close();
			delete sdh_;
		}

		void diag_init(diagnostic_updater::DiagnosticStatusWrapper &stat)
		  {
		    if(isInitialized_ && isDSAInitialized_)
		      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "");
		    else
		      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "");
		    stat.add("Hand initialized", isInitialized_);
		    stat.add("Tactile iInitialized", isDSAInitialized_);
		  }

		/*!
		* \brief Initializes node to get parameters, subscribe and publish to topics.
		*/
		bool init()
		{
			// initialize member variables
			isInitialized_ = false;
			isDSAInitialized_ = false;
			hasNewGoal_ = false;

			// implementation of topics to publish
			topicPub_JointState_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
			topicPub_ControllerState_ = nh_.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("state", 1);
			topicPub_TactileSensor_ = nh_.advertise<cob_sdh::TactileSensor>("tactile_data", 1);

			// pointer to sdh
			sdh_ = new SDH::cSDH(false, false, 0); //(_use_radians=false, bool _use_fahrenheit=false, int _debug_level=0)

			// implementation of service servers
			srvServer_Init_ = nh_.advertiseService("init", &SdhNode::srvCallback_Init, this);
			srvServer_Stop_ = nh_.advertiseService("stop", &SdhNode::srvCallback_Stop, this);
			srvServer_Recover_ = nh_.advertiseService("recover", &SdhNode::srvCallback_Recover, this);
			srvServer_SetOperationMode_ = nh_.advertiseService("set_operation_mode", &SdhNode::srvCallback_SetOperationMode, this);

			// getting hardware parameters from parameter server
			nh_.param("sdhdevicetype", sdhdevicetype_, std::string("PCAN"));
			nh_.param("sdhdevicestring", sdhdevicestring_, std::string("/dev/pcan0"));
			nh_.param("sdhdevicenum", sdhdevicenum_, 0);
			
			nh_.param("dsadevicestring", dsadevicestring_, std::string("/dev/ttyS0"));
			nh_.param("dsadevicenum", dsadevicenum_, 0);
			
			nh_.param("baudrate", baudrate_, 1000000);
			nh_.param("timeout", timeout_, (double)0.04);
			nh_.param("id_read", id_read_, 43);
			nh_.param("id_write", id_write_, 42);

			// get JointNames from parameter server
			ROS_INFO("getting JointNames from parameter server");
			XmlRpc::XmlRpcValue JointNames_param;
			if (nh_.hasParam("JointNames"))
			{
				nh_.getParam("JointNames", JointNames_param);
			}
			else
			{
				ROS_ERROR("Parameter JointNames not set");
				return false;
			}
			DOF_ = JointNames_param.size();
			JointNames_.resize(DOF_);
			for (int i = 0; i<DOF_; i++ )
			{
				JointNames_[i] = (std::string)JointNames_param[i];
			}
			std::cout << "JointNames = " << JointNames_param << std::endl;
			
			// define axes to send to sdh
			axes_.resize(DOF_);
			for(int i=0; i<DOF_; i++)
			{
				axes_[i] = i;
			}
			ROS_INFO("DOF = %d",DOF_);
			
			state_.resize(axes_.size());
			
			return true;
		}

		/*!
		* \brief Executes the callback from the actionlib
		*
		* Set the current goal to aborted after receiving a new goal and write new goal to a member variable. Wait for the goal to finish and set actionlib status to succeeded.
		* \param goal JointTrajectoryGoal
		*/
		void executeCB(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr &goal)
		{			
			ROS_INFO("sdh: executeCB");
			if (!isInitialized_)
			{
				ROS_ERROR("%s: Rejected, sdh not initialized", action_name_.c_str());
				as_.setAborted();
				return;
			}

			while (hasNewGoal_ == true ) usleep(10000);

			// \todo TODO: use joint_names for assigning values
			targetAngles_.resize(DOF_);
			targetAngles_[0] = goal->trajectory.points[0].positions[2]*180.0/pi_; // sdh_knuckle_joint
			targetAngles_[1] = goal->trajectory.points[0].positions[5]*180.0/pi_; // sdh_finger22_joint
			targetAngles_[2] = goal->trajectory.points[0].positions[6]*180.0/pi_; // sdh_finger23_joint
			targetAngles_[3] = goal->trajectory.points[0].positions[0]*180.0/pi_; // sdh_thumb2_joint
			targetAngles_[4] = goal->trajectory.points[0].positions[1]*180.0/pi_; // sdh_thumb3_joint
			targetAngles_[5] = goal->trajectory.points[0].positions[3]*180.0/pi_; // sdh_finger12_joint
			targetAngles_[6] = goal->trajectory.points[0].positions[4]*180.0/pi_; // sdh_finger13_joint
			ROS_INFO("received new position goal: [['sdh_thumb_2_joint', 'sdh_thumb_3_joint', 'sdh_knuckle_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint']] = [%f,%f,%f,%f,%f,%f,%f,%f]",goal->trajectory.points[0].positions[0],goal->trajectory.points[0].positions[1],goal->trajectory.points[0].positions[2],goal->trajectory.points[0].positions[3],goal->trajectory.points[0].positions[4],goal->trajectory.points[0].positions[5],goal->trajectory.points[0].positions[6]);
		
			hasNewGoal_ = true;
			
			usleep(500000); // needed sleep until sdh starts to change status from idle to moving
			
			bool finished = false;
			while(finished == false)
			{
				if (as_.isNewGoalAvailable())
				{
					ROS_WARN("%s: Aborted", action_name_.c_str());
					as_.setAborted();
					return;
				}
				for ( size_t i = 0; i < state_.size(); i++ )
		 		{
		 			ROS_DEBUG("state[%d] = %d",i,state_[i]);
		 			if (state_[i] == 0)
		 			{
		 				finished = true;
		 			}
		 			else
		 			{	
		 				finished = false;
		 			}
		 		}
		 		usleep(10000);
				//feedback_ = 
				//as_.send feedback_
			}

			// set the action state to succeeded			
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			//result_.result.data = "succesfully received new goal";
			//result_.success = 1;
			//as_.setSucceeded(result_);
			as_.setSucceeded();
		}

		/*!
		* \brief Executes the service callback for init.
		*
		* Connects to the hardware and initialized it.
		* \param req Service request
		* \param res Service response
		*/
		bool srvCallback_Init(	cob_srvs::Trigger::Request &req,
								cob_srvs::Trigger::Response &res )
		{

			if (isInitialized_ == false)
			{
				//Init Hand connection	
				
				try
				{
					if(sdhdevicetype_.compare("RS232")==0)
					{
						sdh_->OpenRS232( sdhdevicenum_, 115200, 1, sdhdevicestring_.c_str());
						ROS_INFO("Initialized RS232 for SDH");
						isInitialized_ = true;
					}
					if(sdhdevicetype_.compare("PCAN")==0)
					{
						ROS_INFO("Starting initializing PEAKCAN");
						sdh_->OpenCAN_PEAK(baudrate_, timeout_, id_read_, id_write_, sdhdevicestring_.c_str());
						ROS_INFO("Initialized PEAK CAN for SDH");
						isInitialized_ = true;
					}
					if(sdhdevicetype_.compare("ESD")==0)
					{
						ROS_INFO("Starting initializing ESD");
						if(strcmp(sdhdevicestring_.c_str(), "/dev/can0") == 0)
						{
							ROS_INFO("Initializing ESD on device %s",sdhdevicestring_.c_str());
							sdh_->OpenCAN_ESD(0, baudrate_, timeout_, id_read_, id_write_ );
						}
						else if(strcmp(sdhdevicestring_.c_str(), "/dev/can1") == 0)
						{
							ROS_INFO("Initializin ESD on device %s",sdhdevicestring_.c_str());
							sdh_->OpenCAN_ESD(1, baudrate_, timeout_, id_read_, id_write_ );
						}
						else
						{
							ROS_ERROR("Currently only support for /dev/can0 and /dev/can1");
							res.success.data = false;
							res.error_message.data = "Currently only support for /dev/can0 and /dev/can1";
							return true;
						}
						ROS_INFO("Initialized ESDCAN for SDH");	
						isInitialized_ = true;
					}
				}
				catch (SDH::cSDHLibraryException* e)
				{
					ROS_ERROR("An exception was caught: %s", e->what());
					res.success.data = false;
					res.error_message.data = e->what();
					delete e;
					return true;
				}
				
				//Init tactile data
				try
				{
					dsa_ = new SDH::cDSA(0, dsadevicenum_, dsadevicestring_.c_str());
					//dsa_->SetFramerate( 0, true, false );
					dsa_->SetFramerate( 1, true );
					ROS_INFO("Initialized RS232 for DSA Tactile Sensors on device %s",dsadevicestring_.c_str());
					// ROS_INFO("Set sensitivity to 1.0");
					// for(int i=0; i<6; i++)
					// 	dsa_->SetMatrixSensitivity(i, 1.0);
					isDSAInitialized_ = true;
				}
				catch (SDH::cSDHLibraryException* e)
				{
					isDSAInitialized_ = false;
					ROS_ERROR("An exception was caught: %s", e->what());
					res.success.data = false;
					res.error_message.data = e->what();
					delete e;
					return true;
				}
				
			}
			else
			{
				ROS_WARN("...sdh already initialized...");
				res.success.data = true;
				res.error_message.data = "sdh already initialized";
			}
			
			res.success.data = true;
			return true;
		}

		/*!
		* \brief Executes the service callback for stop.
		*
		* Stops all hardware movements.
		* \param req Service request
		* \param res Service response
		*/
		bool srvCallback_Stop(cob_srvs::Trigger::Request &req,
				cob_srvs::Trigger::Response &res )
		{
			ROS_INFO("Stopping sdh");

			// stopping all arm movements
			try
			{
				sdh_->Stop();
			}
			catch (SDH::cSDHLibraryException* e)
			{
				ROS_ERROR("An exception was caught: %s", e->what());
				delete e;
			}

		ROS_INFO("Stopping sdh succesfull");
		res.success.data = true;
		return true;
	}

	/*!
	* \brief Executes the service callback for recover.
	*
	* Recovers the hardware after an emergency stop.
	* \param req Service request
	* \param res Service response
	*/
	bool srvCallback_Recover(cob_srvs::Trigger::Request &req,
					cob_srvs::Trigger::Response &res )
	{
		ROS_WARN("Service recover not implemented yet");
		res.success.data = true;
		res.error_message.data = "Service recover not implemented yet";
		return true;
	}
	
	/*!
	* \brief Executes the service callback for set_operation_mode.
	*
	* Changes the operation mode.
	* \param req Service request
	* \param res Service response
	*/
	bool srvCallback_SetOperationMode(cob_srvs::SetOperationMode::Request &req,
					cob_srvs::SetOperationMode::Response &res )
	{
		ROS_INFO("Set operation mode to [%s]", req.operation_mode.data.c_str());
		nh_.setParam("OperationMode", req.operation_mode.data.c_str());
		res.success.data = true;
		return true;
	}

	/*!
	* \brief Main routine to update sdh.
	*
	* Sends target to hardware and reads out current configuration.
	*/
	void updateSdh()
	{
		ROS_DEBUG("updateJointState");
		updater_.update();
		if (isInitialized_ == true)
		{
			if (hasNewGoal_ == true)
			{
				// stop sdh first when new goal arrived
				try
				{
					sdh_->Stop();
				}
				catch (SDH::cSDHLibraryException* e)
				{
					ROS_ERROR("An exception was caught: %s", e->what());
					delete e;
				}
		
				std::string operationMode;
				nh_.getParam("OperationMode", operationMode);
				if (operationMode == "position")
				{
					ROS_DEBUG("moving sdh in position mode");

					try
					{
						sdh_->SetAxisTargetAngle( axes_, targetAngles_ );
						sdh_->MoveHand(false);
					}
					catch (SDH::cSDHLibraryException* e)
					{
						ROS_ERROR("An exception was caught: %s", e->what());
						delete e;
					}
				}
				else if (operationMode == "velocity")
				{
					ROS_DEBUG("moving sdh in velocity mode");
					//sdh_->MoveVel(goal->trajectory.points[0].velocities);
					ROS_WARN("Moving in velocity mode currently disabled");
				}
				else if (operationMode == "effort")
				{
					ROS_DEBUG("moving sdh in effort mode");
					//sdh_->MoveVel(goal->trajectory.points[0].velocities);
					ROS_WARN("Moving in effort mode currently disabled");
				}
				else
				{
					ROS_ERROR("sdh neither in position nor in velocity nor in effort mode. OperationMode = [%s]", operationMode.c_str());
				}
				
				hasNewGoal_ = false;
			}

	 		// read and publish joint angles and velocities
			std::vector<double> actualAngles;
			try
			{
				actualAngles = sdh_->GetAxisActualAngle( axes_ );
			}
			catch (SDH::cSDHLibraryException* e)
			{
				ROS_ERROR("An exception was caught: %s", e->what());
				delete e;
			}
			std::vector<double> actualVelocities;
			try
			{
				actualVelocities = sdh_->GetAxisActualVelocity( axes_ );
			}
			catch (SDH::cSDHLibraryException* e)
			{
				ROS_ERROR("An exception was caught: %s", e->what());
				delete e;
			}
			
			ROS_DEBUG("received %d angles from sdh",actualAngles.size());
			
			// create joint_state message
			sensor_msgs::JointState msg;
			msg.header.stamp = ros::Time::now();
			msg.name.resize(DOF_);
			msg.position.resize(DOF_);
			msg.velocity.resize(DOF_);

			// set joint names and map them to angles
			msg.name = JointNames_;
			//['sdh_thumb_2_joint', 'sdh_thumb_3_joint', 'sdh_knuckle_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint']
			// pos
			msg.position[0] = actualAngles[3]*pi_/180.0; // sdh_thumb_2_joint
			msg.position[1] = actualAngles[4]*pi_/180.0; // sdh_thumb_3_joint
			msg.position[2] = actualAngles[0]*pi_/180.0; // sdh_knuckle_joint
			msg.position[3] = actualAngles[5]*pi_/180.0; // sdh_finger_12_joint
			msg.position[4] = actualAngles[6]*pi_/180.0; // sdh_finger_13_joint
			msg.position[5] = actualAngles[1]*pi_/180.0; // sdh_finger_22_joint
			msg.position[6] = actualAngles[2]*pi_/180.0; // sdh_finger_23_joint
			// vel			
			msg.velocity[0] = actualVelocities[3]*pi_/180.0; // sdh_thumb_2_joint
			msg.velocity[1] = actualVelocities[4]*pi_/180.0; // sdh_thumb_3_joint
			msg.velocity[2] = actualVelocities[0]*pi_/180.0; // sdh_knuckle_joint
			msg.velocity[3] = actualVelocities[5]*pi_/180.0; // sdh_finger_12_joint
			msg.velocity[4] = actualVelocities[6]*pi_/180.0; // sdh_finger_13_joint
			msg.velocity[5] = actualVelocities[1]*pi_/180.0; // sdh_finger_22_joint
			msg.velocity[6] = actualVelocities[2]*pi_/180.0; // sdh_finger_23_joint

			// publish message
			topicPub_JointState_.publish(msg); 

			// read sdh status
			state_ = sdh_->GetAxisActualState(axes_);
		}
		else
		{
			ROS_DEBUG("sdh not initialized");
		}
	}

	/*!
	* \brief Main routine to update dsa.
	*
	* Reads out current values.
	*/
	void updateDsa()
	{
		ROS_DEBUG("updateTactileData");

		if(isDSAInitialized_)
		{
			// read tactile data
			for(int i=0; i<7; i++)
			{
				try
				{
					//dsa_->SetFramerate( 0, true, true );
					dsa_->UpdateFrame();
				}
				catch (SDH::cSDHLibraryException* e)
				{
					ROS_ERROR("An exception was caught: %s", e->what());
					delete e;
				}
			}

			cob_sdh::TactileSensor msg;
			msg.header.stamp = ros::Time::now();
			int m, x, y;
			msg.tactile_matrix.resize(dsa_->GetSensorInfo().nb_matrices);
			for ( m = 0; m < dsa_->GetSensorInfo().nb_matrices; m++ )
			{
				cob_sdh::TactileMatrix &tm = msg.tactile_matrix[m];
				tm.matrix_id = m;
				tm.cells_x = dsa_->GetMatrixInfo( m ).cells_x;
				tm.cells_y = dsa_->GetMatrixInfo( m ).cells_y;
				tm.tactile_array.resize(tm.cells_x * tm.cells_y);
				for ( y = 0; y < tm.cells_y; y++ )
				{
					for ( x = 0; x < tm.cells_x; x++ )
						tm.tactile_array[tm.cells_x*y + x] = dsa_->GetTexel( m, x, y );
				}
			}
			//publish matrix
			topicPub_TactileSensor_.publish(msg);
		}
	}	 
}; //SdhNode

/*!
* \brief Main loop of ROS node.
*
* Running with a specific frequency defined by loop_rate.
*/
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "cob_sdh");

	SdhNode sdh_node("joint_trajectory_action");
	if (!sdh_node.init()) return 0;
	
	ROS_INFO("...sdh node running...");

	sleep(1);
	ros::Rate loop_rate(5); // Hz
	while(sdh_node.nh_.ok())
	{
		// publish JointState
		sdh_node.updateSdh();
		
		// publish TactileData
		sdh_node.updateDsa();
		
		// sleep and waiting for messages, callbacks
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
