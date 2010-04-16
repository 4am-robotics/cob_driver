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
 * ROS package name: sdh
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: Jan 2010
 * ToDo:
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

//#define USE_ESD

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
#include <urdf/model.h>
#include <actionlib/server/simple_action_server.h>

// ROS message includes
#include <cob_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>
#include <cob_actions/JointCommandAction.h>
// #include <cob_msgs/TactileMatrix.h>

// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>

// external includes
#include <cob_sdh/sdh.h>
#include <cob_sdh/dsa.h>
#include <cob_sdh/util.h>
#include <cob_sdh/sdhlibrary_settings.h>
#include <cob_sdh/basisdef.h>

//########################
//#### sdh node class ####
class SdhNode
{
	public:
		// create a handle for this node, initialize node
		ros::NodeHandle nh_;
	private:
		// declaration of topics to publish
		ros::Publisher topicPub_JointState_;
		//ros::Publisher topicPub_TactileMatrix_;

		// service servers
		ros::ServiceServer srvServer_Init_;
        ros::ServiceServer srvServer_Stop_;
        ros::ServiceServer srvServer_SetOperationMode_;

        // action lib server
		actionlib::SimpleActionServer<cob_actions::JointCommandAction> as_;
		std::string action_name_;
		cob_actions::JointCommandFeedback feedback_;
		cob_actions::JointCommandResult result_;

		// service clients
		//--

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
		int DOF_HW_,DOF_ROS_;
		double pi_;
		
		cob_msgs::JointCommand command_;
		
		std::vector<std::string> JointNames_;
		std::vector<std::string> JointNamesAll_;
		std::vector<int> axes_;
		std::vector<double> targetAngles_; // in degrees
		bool hasNewGoal_;
		
	public:
		// Constructor
		SdhNode(std::string name):
			as_(nh_, name, boost::bind(&SdhNode::executeCB, this, _1)),
			action_name_(name)
		{
			pi_ = 3.1415926;
		}

		// Destructor
		~SdhNode() 
		{
			sdh_->Close();
			delete sdh_;
		}
		
		bool init()
		{
			// initialize global variables
			isInitialized_ = false;
			isDSAInitialized_ = false;
			hasNewGoal_ = false;

			// implementation of topics to publish
			topicPub_JointState_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
			//topicPub_TactileMatrix_ = nh_.advertise<cob_msgs::TactileMatrix>("tactile_data", 1);

			// pointer to sdh
			sdh_ = new SDH::cSDH(false, false, 0); //(_use_radians=false, bool _use_fahrenheit=false, int _debug_level=0)

			// implementation of service servers
			srvServer_Init_ = nh_.advertiseService("Init", &SdhNode::srvCallback_Init, this);
            srvServer_Stop_ = nh_.advertiseService("Stop", &SdhNode::srvCallback_Stop, this);
            srvServer_SetOperationMode_ = nh_.advertiseService("SetOperationMode", &SdhNode::srvCallback_SetOperationMode, this);
            
            // getting harware parameters from parameter server
#ifdef USE_ESD
			nh_.param("sdhdevicetype", sdhdevicetype_, std::string("ESD"));
			nh_.param("sdhdevicestring", sdhdevicestring_, std::string("/dev/can0"));
#else
			nh_.param("sdhdevicetype", sdhdevicetype_, std::string("PEAK"));
			nh_.param("sdhdevicestring", sdhdevicestring_, std::string("/dev/pcan0"));
#endif
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
			DOF_HW_ = JointNames_param.size(); // DOFs of sdh, NOTE: hardware has 8 DOFs, but two cuppled ones; joints palm_finger11 and palm_finger21 are actuated synchronously
			JointNames_.resize(DOF_HW_);
			for (int i = 0; i<DOF_HW_; i++ )
			{
				JointNames_[i] = (std::string)JointNames_param[i];
			}
			std::cout << "JointNames = " << JointNames_param << std::endl;

            // get JointNamesAll from parameter server			
			ROS_INFO("getting JointNamesAll from parameter server");
			XmlRpc::XmlRpcValue JointNamesAll_param;
			if (nh_.hasParam("JointNamesAll"))
			{
				nh_.getParam("JointNamesAll", JointNamesAll_param);
			}
			else
			{
				ROS_ERROR("Parameter JointNamesAll not set");
				return false;
			}
			DOF_ROS_ = JointNamesAll_param.size(); // DOFs of sdh, NOTE: hardware has 8 DOFs, but two cuppled ones; joints palm_finger11 and palm_finger21 are actuated synchronously
			JointNamesAll_.resize(DOF_ROS_);
			for (int i = 0; i<DOF_ROS_; i++ )
			{
				JointNamesAll_[i] = (std::string)JointNamesAll_param[i];
			}
			std::cout << "JointNamesAll = " << JointNamesAll_param << std::endl;
			
			// define axes to send to sdh
			axes_.resize(DOF_HW_);
			for(int i=0; i<DOF_HW_; i++)
			{
				axes_[i] = i;
			}
			ROS_INFO("DOF_HW = %d, DOF_ROS = %d",DOF_HW_,DOF_ROS_);
			
			state_.resize(axes_.size());
			
			return true;
		}

		void executeCB(const cob_actions::JointCommandGoalConstPtr &goal)
		{			
			if (!isInitialized_)
			{
				ROS_ERROR("%s: Rejected, sdh not initialized", action_name_.c_str());
				as_.setAborted();
				return;
			}

			while (hasNewGoal_ == true ) usleep(10000);

			targetAngles_.resize(DOF_HW_);
			targetAngles_[0] = goal->command.positions[3]*180.0/pi_; // joint_palm_finger11
			targetAngles_[1] = goal->command.positions[7]*180.0/pi_; // joint_finger21_finger22
			targetAngles_[2] = goal->command.positions[8]*180.0/pi_; // joint_finger22_finger23
			targetAngles_[3] = goal->command.positions[1]*180.0/pi_; // joint_thumb1_thumb2
			targetAngles_[4] = goal->command.positions[2]*180.0/pi_; // joint_thumb2_thumb3
			targetAngles_[5] = goal->command.positions[4]*180.0/pi_; // joint_finger11_finger12
			targetAngles_[6] = goal->command.positions[5]*180.0/pi_; // joint_finger12_finger13
			std::cout << "received new position goal: " << targetAngles_[0] << " , " << targetAngles_[1] << " , " << targetAngles_[2] << " , " << targetAngles_[3] << " , " << targetAngles_[4] << " , " << targetAngles_[5] << " , " << targetAngles_[6] << std::endl;
		
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
				for ( int i = 0; i < state_.size(); i++ )
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
			result_.result.data = "succesfully received new goal";
			result_.success = 1;
			as_.setSucceeded(result_);
		}

		// service callback functions
		// function will be called when a service is querried
		bool srvCallback_Init(cob_srvs::Trigger::Request &req,
				cob_srvs::Trigger::Response &res )
		{
			ROS_INFO("Initializing sdh");

			if (isInitialized_ == false)
			{
				try
				{
					if(sdhdevicetype_.compare("RS232")==0)
					{
						sdh_->OpenRS232( sdhdevicenum_, 115200, 1, sdhdevicestring_.c_str());
						ROS_INFO("Initialized RS232 for SDH");
						isInitialized_ = true;
					}
					if(sdhdevicetype_.compare("PEAK")==0)
					{
						ROS_INFO("Starting initializing PEAKCAN");
						sdh_->OpenCAN_PEAK(baudrate_, timeout_, id_read_, id_write_, sdhdevicestring_.c_str());
						ROS_INFO("Initialized PEAK CAN for SDH");
						isInitialized_ = true;
					}
					if(sdhdevicetype_.compare("ESD")==0)
					{
						ROS_INFO("Starting init ESD");
						sdh_->OpenCAN_ESD(0, baudrate_, timeout_, id_read_, id_write_ );
						ROS_INFO("Initialized ESDCAN for SDH");
						isInitialized_ = true;
					}
				}
				catch (SDH::cSDHLibraryException* e)
				{
					ROS_ERROR("An exception was caught: %s", e->what());
					delete e;
				}
			}
			else
			{
				ROS_ERROR("...sdh already initialized...");		        
				res.success = 1;
				res.errorMessage.data = "sdh already initialized";
			}
			
			return true;
		}

		bool srvCallback_Stop(cob_srvs::Trigger::Request &req,
                              cob_srvs::Trigger::Response &res )
        {
       	    ROS_INFO("Stopping sdh");
        	        	
            // stopping all arm movements
            sdh_->Stop();
           	ROS_INFO("Stopping sdh succesfull");
           	res.success = 0; // 0 = true, else = false
            return true;
        }

        bool srvCallback_SetOperationMode(cob_srvs::SetOperationMode::Request &req,
                                          cob_srvs::SetOperationMode::Response &res )
        {
        	ROS_INFO("Set operation mode to [%s]", req.operationMode.data.c_str());
            nh_.setParam("OperationMode", req.operationMode.data.c_str());
            res.success = 0; // 0 = true, else = false
            return true;
        }

		void updateSdh()
        {
        	ROS_DEBUG("updateJointState");
        	
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
						}
						catch (SDH::cSDHLibraryException* e)
						{
							ROS_ERROR("An exception was caught: %s", e->what());
							delete e;
						}

						try
						{
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
        	
        		// read and publish joint angles
				std::vector<double> actualAngles;
				actualAngles = sdh_->GetAxisActualAngle( axes_ );
				
				ROS_DEBUG("received %d angles from sdh",actualAngles.size());
				
		        // create joint_state message
		        sensor_msgs::JointState msg;
				msg.header.stamp = ros::Time::now();
				msg.name.resize(DOF_ROS_);
				msg.position.resize(DOF_ROS_);

				// set joint names and map them to angles
				msg.name = JointNamesAll_;
				//std::cout << actualAngles[0] << " , " << actualAngles[1] << " , " << actualAngles[2] << " , " << actualAngles[3] << " , " << actualAngles[4] << " , " << actualAngles[5] << " , " << actualAngles[6] << std::endl;
				msg.position[0] = 0.0; // joint_palm_thumb1
				msg.position[1] = actualAngles[3]*pi_/180.0; // joint_thumb1_thumb2
				msg.position[2] = actualAngles[4]*pi_/180.0; // joint_thumb2_thumb3
				msg.position[3] = actualAngles[0]*pi_/180.0; // joint_palm_finger11
				msg.position[4] = actualAngles[5]*pi_/180.0; // joint_finger11_finger12
				msg.position[5] = actualAngles[6]*pi_/180.0; // joint_finger12_finger13
				msg.position[6] = actualAngles[0]*pi_/180.0; // joint_palm_finger21
				msg.position[7] = actualAngles[1]*pi_/180.0; // joint_finger21_finger22
				msg.position[8] = actualAngles[2]*pi_/180.0; // joint_finger22_finger23
		            
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
		
		/* ////
		   void updateTactileData()
		   {
		   ROS_INFO("updateTactileData");
		   cob_msgs::TactileMatrix msg;
		   if(isDSAInitialized)
		   {
		   dsa->UpdateFrame();
		   unsigned int m, x, y;
		   for ( m = 0; m < dsa->GetSensorInfo().nb_matrices; m++ )
		   {
		   msg.maxtrix_id = m;
		   int cells_y = dsa->GetMatrixInfo( m ).cells_y;
		   int cells_x = dsa->GetMatrixInfo( m ).cells_x;
		   msg.cells_y = cells_y;
		   msg.cells_x = cells_x;
		   msg.texel_data.resize((cells_y*cells_x)+1);
		   for ( y = 0; y < cells_y; y++ )
		   {
		   for ( x = 0; x < cells_x; x++ )
		   {
		   msg.texel_data[(y+1)*(x+1)] = dsa->GetTexel( m, x, y );
		//std::cout << std::setw( 4 ) << dsa->GetTexel( m, x, y ) << " ";
		}
		//std::cout << "\n";
		}
		//std::cout << "\n\n";
		//publish matrix
		topicPub_TactileMatrix.publish(msg);
		}
		}
		}
		 */
		 
}; //SdhNode

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "cob_sdh");

	SdhNode sdh_node("JointCommand");
	if (!sdh_node.init()) return 0;
	
	ROS_INFO("...sdh node running...");

	sleep(1);
	ros::Rate loop_rate(5); // Hz
	while(sdh_node.nh_.ok())
	{
		// publish JointState
		sdh_node.updateSdh();
		////sdh_node.updateTactileData();
		
		// sleep and waiting for messages, callbacks    
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
