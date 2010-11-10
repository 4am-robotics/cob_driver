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
 * ROS stack name: cob3_driver
 * ROS package name: cob_camera_axis
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Ulrich Reiser, email:ulrich.reiser@ipa.fhg.de
 *
 * Date of creation: Jan 2010
 * ToDo:
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
#include <urdf/model.h>
#include <actionlib/server/simple_action_server.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

// ROS service includes
#include <cob_srvs/Trigger.h>

// external includes
#include <cob_camera_axis/ElmoCtrl.h>

//####################
//#### node class ####
class NodeClass
{
	//
	public:
	// create a handle for this node, initialize node
	ros::NodeHandle n_;
		
	// declaration of topics to publish
	ros::Publisher topicPub_JointState_;
	
	// declaration of topics to subscribe, callback is called for new messages arriving
	ros::Subscriber topicSub_JointCommand_;
	
	// declaration of service servers
	ros::ServiceServer srvServer_Init_;
	ros::ServiceServer srvServer_Stop_;
	ros::ServiceServer srvServer_Recover_;
		
	// declaration of service clients
	//--

	// action lib server
	actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> as_;
	std::string action_name_;
	pr2_controllers_msgs::JointTrajectoryFeedback feedback_;
	pr2_controllers_msgs::JointTrajectoryResult result_;
	
	// global variables
	ElmoCtrl * CamAxis_;
	ElmoCtrlParams* CamAxisParams_;
	std::string CanDevice_;
	std::string CanIniFile_;
	int CanBaudrate_;
	int HomingDir_;
	double MaxVel_;
	int ModID_;
	std::string operationMode_;
	double LowerLimit_;
	double UpperLimit_; 
	double Offset_;
	std::string JointName_;
	bool isInitialized_;
	bool finished_;
	double ActualPos_;
	double ActualVel_;
	trajectory_msgs::JointTrajectory traj_;
	trajectory_msgs::JointTrajectoryPoint traj_point_;
	int traj_point_nr_;

	// Constructor
	NodeClass(std::string name):
		as_(n_, name, boost::bind(&NodeClass::executeCB, this, _1)),
		action_name_(name)
	{
		isInitialized_ = false;
		ActualPos_=0.0;
		ActualVel_=0.0;

		CamAxis_ = new ElmoCtrl();
		CamAxisParams_ = new ElmoCtrlParams();

		// implementation of topics to publish
		topicPub_JointState_ = n_.advertise<sensor_msgs::JointState>("/joint_states", 1);
		
		// implementation of topics to subscribe
		
		// implementation of service servers
		srvServer_Init_ = n_.advertiseService("init", &NodeClass::srvCallback_Init, this);
		srvServer_Stop_ = n_.advertiseService("stop", &NodeClass::srvCallback_Stop, this);
		srvServer_Recover_ = n_.advertiseService("recover", &NodeClass::srvCallback_Recover, this);
		
		// implementation of service clients
		//--

		// read parameters from parameter server
		CanDevice_ = "PCAN";
		CanBaudrate_ = 500;

		n_.getParam("CanDevice", CanDevice_);
		n_.getParam("CanBaudrate", CanBaudrate_);
		n_.getParam("HomingDir", HomingDir_);
		n_.getParam("ModId",ModID_);
		n_.getParam("JointName",JointName_);
		n_.getParam("CanIniFile",CanIniFile_);
		n_.getParam("OperationMode",operationMode_);
		
		//n_.param<double>("MaxVel", MaxVel_, 2.0); -> from urdf
		ROS_INFO("CanDevice=%s, CanBaudrate=%d,ModID=%d",CanDevice_.c_str(),CanBaudrate_,ModID_);
		
		
		// load parameter server string for robot/model
		std::string param_name = "/robot_description";
		std::string full_param_name;
		std::string xml_string;
		n_.searchParam(param_name,full_param_name);
		n_.getParam(full_param_name.c_str(),xml_string);
		ROS_INFO("full_param_name=%s",full_param_name.c_str());
		if (xml_string.size()==0)
		{
			ROS_ERROR("Unable to load robot model from param server robot_description\n");
			exit(2);
		}
		ROS_DEBUG("%s content\n%s", full_param_name.c_str(), xml_string.c_str());
		
		// extract limits and velocitys from urdf model
		// The urdf model can be found in torso_cob3-1.urdf -> joint_head_eyes
		urdf::Model model;
		if (!model.initString(xml_string))
		{
			ROS_ERROR("Failed to parse urdf file");
			exit(2);
		}
		ROS_INFO("Successfully parsed urdf file");

		// get LowerLimits out of urdf model
		LowerLimit_ = model.getJoint(JointName_.c_str())->limits->lower;
			//std::cout << "LowerLimits[" << JointNames[i].c_str() << "] = " << LowerLimits[i] << std::endl;
		CamAxisParams_->SetLowerLimit(LowerLimit_);

		// get UpperLimits out of urdf model
		UpperLimit_ = model.getJoint(JointName_.c_str())->limits->upper;
			//std::cout << "LowerLimits[" << JointNames[i].c_str() << "] = " << LowerLimits[i] << std::endl;
		CamAxisParams_->SetUpperLimit(UpperLimit_);

		// get Offset out of urdf model
		Offset_ = model.getJoint(JointName_.c_str())->calibration->rising.get()[0];
			//std::cout << "Offset[" << JointNames[i].c_str() << "] = " << Offsets[i] << std::endl;
		CamAxisParams_->SetAngleOffset(Offset_);
		
		// get velocitiy out of urdf model
		MaxVel_ = model.getJoint(JointName_.c_str())->limits->velocity;
		ROS_INFO("Successfully read limits from urdf");


		//initializing and homing of camera_axis		
		CamAxisParams_->SetCanIniFile(CanIniFile_);
		CamAxisParams_->SetHomingDir(HomingDir_);
		CamAxisParams_->SetMaxVel(MaxVel_);

		CamAxisParams_->Init(CanDevice_, CanBaudrate_, ModID_);
		

	}
	
	// Destructor
	~NodeClass() 
	{
		delete CamAxis_;
	}

	void executeCB(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr &goal) {
		if(isInitialized_) {	
			ROS_INFO("Received new goal trajectory with %d points",goal->trajectory.points.size());
			// saving goal into local variables
			traj_ = goal->trajectory;
			traj_point_nr_ = 0;
			traj_point_ = traj_.points[traj_point_nr_];
			finished_ = false;
			
			// stoping axis to prepare for new trajectory
			CamAxis_->Stop();

			// check that preempt has not been requested by the client
			if (as_.isPreemptRequested())
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
				// set the action state to preempted
				as_.setPreempted();
			}
			
			usleep(2000000); // needed sleep until powercubes starts to change status from idle to moving
			
			while(finished_ == false)
			{
				if (as_.isNewGoalAvailable())
				{
					ROS_WARN("%s: Aborted", action_name_.c_str());
					as_.setAborted();
					return;
				}
		   		usleep(10000);
				//feedback_ = 
				//as_.send feedback_
			}

			// set the action state to succeed			
			//result_.result.data = "executing trajectory";
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			// set the action state to succeeded
			as_.setSucceeded(result_);
		
		} else {
			as_.setAborted();
			ROS_WARN("Camera_axis not initialized yet!");
		}
	}

	// service callback functions
	// function will be called when a service is querried
	bool srvCallback_Init(cob_srvs::Trigger::Request &req,
				  cob_srvs::Trigger::Response &res )
	{
		if (isInitialized_ == false) {
			ROS_INFO("...initializing camera axis...");
			// init powercubes 
			if (CamAxis_->Init(CamAxisParams_))
			{
				ROS_INFO("Initializing of camera axis succesful");
					isInitialized_ = true;
				res.success = 0; // 0 = true, else = false
			}
			else
			{
				ROS_ERROR("Initializing camera axis not succesful \n");
				//res.success = 1; // 0 = true, else = false
				//res.errorMessage.data = PCube->getErrorMessage();
			}
			}
			else
			{
				ROS_ERROR("...camera axis already initialized...");			
				res.success = 1;
				res.errorMessage.data = "camera axis already initialized";
		}
		
		return true;
	}

	bool srvCallback_Stop(cob_srvs::Trigger::Request &req,
				  cob_srvs::Trigger::Response &res )
	{
	ROS_INFO("Stopping camera axis");
	
	// stopping all arm movements
	if (CamAxis_->Stop()) {
		ROS_INFO("Stopping camera axis succesful");
		res.success = 0; // 0 = true, else = false
	}
	else {
		ROS_ERROR("Stopping camera axis not succesful. error");
	}

	return true;
	}
	
	bool srvCallback_Recover(cob_srvs::Trigger::Request &req,
				  	 cob_srvs::Trigger::Response &res )
	{
		if (isInitialized_ == true) {
			ROS_INFO("Recovering camera axis");
			
			// stopping all arm movements
			if (CamAxis_->RecoverAfterEmergencyStop()) {
				ROS_INFO("Recovering camera axis succesful");
				res.success = 0; // 0 = true, else = false
			} else {
				ROS_ERROR("Recovering camera axis not succesful. error");
			}
		} else {
			ROS_ERROR("...camera axis already recovered...");			
		}

		return true;
	}

	// other function declarations
	void updateCommands()
		{
			if (isInitialized_ == true)
			{
			    if (operationMode_ == "position")
			    {
				    ROS_DEBUG("moving head_axis in position mode");
			    	if (ActualVel_ < 0.002)
			    	{
				    	//feedback_.isMoving = false;
				    	
				    	ROS_DEBUG("next point is %d from %d",traj_point_nr_,traj_.points.size());
				    	
				    	if (traj_point_nr_ < traj_.points.size())
				    	{
				    		// if axis is not moving and not reached last point of trajectory, then send new target point
				    		ROS_INFO("...moving to trajectory point[%d]",traj_point_nr_);
					    	traj_point_ = traj_.points[traj_point_nr_];
					    	CamAxis_->setGearPosVelRadS(traj_point_.positions[0], MaxVel_);
					    	usleep(100000);
					    	CamAxis_->m_Joint->requestPosVel();
				    		traj_point_nr_++;
					    	//feedback_.isMoving = true;
					    	//feedback_.pointNr = traj_point_nr;
	    					//as_.publishFeedback(feedback_);
					    }
					    else
					    {
					    	ROS_DEBUG("...reached end of trajectory");
					    	finished_ = true;
					    }
					}
					else
					{
						ROS_INFO("...axis still moving to point[%d]",traj_point_nr_);
					}
			    }
			    else if (operationMode_ == "velocity")
			    {
			        ROS_WARN("Moving in velocity mode currently disabled");
			    }
			    else
			    {
			        ROS_ERROR("axis neither in position nor in velocity mode. OperationMode = [%s]", operationMode_.c_str());
			    }
			}
			else
			{
				ROS_DEBUG("axis not initialized");
			}
		}
	
	void publishJointState()
	{
		if (isInitialized_ == true) {			
			// create message
			int DOF = 1;

			CamAxis_->evalCanBuffer();
			CamAxis_->getGearPosVelRadS(&ActualPos_,&ActualVel_);
			CamAxis_->m_Joint->requestPosVel();

			sensor_msgs::JointState msg;
			msg.header.stamp = ros::Time::now();
			msg.name.resize(DOF);
			msg.position.resize(DOF);
			msg.velocity.resize(DOF);
			
			msg.name[0] = JointName_;
			msg.position[0] = ActualPos_;
			msg.velocity[0] = ActualVel_;


			std::cout << "Joint " << msg.name[0] <<": pos="<<  msg.position[0] << " vel=" << msg.velocity[0] << std::endl;
				
			// publish message
			topicPub_JointState_.publish(msg);
		}
	}

}; //NodeClass


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "cob_camera_axis");
	
	// create nodeClass
	NodeClass nodeClass("joint_trajectory_action");
 
	// main loop
 	ros::Rate loop_rate(5); // Hz
	while(nodeClass.n_.ok()) {
	  
		// publish JointState
		nodeClass.publishJointState();

		// update commands
		nodeClass.updateCommands();

		// sleep and waiting for messages, callbacks 
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

