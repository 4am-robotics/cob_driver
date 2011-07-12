/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2011 \n
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
 *   ROS package name: cob_trajectory_controller
 *
 * \author
 *   Author: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 * \author
 *   Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 *
 * \date Date of creation: March 2011
 *
 * \brief
 *   Implementation of ROS node for powercube_chain.
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



#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <brics_actuator/JointVelocities.h>
#include <cob_trajectory_controller/genericArmCtrl.h>
// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>



#define HZ 100

class cob_trajectory_controller
{
private:
    ros::NodeHandle n_;
    ros::Publisher joint_pos_pub_;
    ros::Publisher joint_vel_pub_;
    ros::Subscriber controller_state_;
  ros::Subscriber operation_mode_;
  ros::ServiceServer srvServer_Stop_;
  ros::ServiceClient srvClient_SetOperationMode;
    actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> as_;
    std::string action_name_;
  std::string current_operation_mode_;
    bool executing_;
  int watchdog_counter;
    genericArmCtrl* traj_generator_;
    trajectory_msgs::JointTrajectory traj_;
    std::vector<double> q_current, startposition_, joint_distance_;

public:
    cob_trajectory_controller():as_(n_, "joint_trajectory_action", boost::bind(&cob_trajectory_controller::executeTrajectory, this, _1), true),
    action_name_("joint_trajectory_action")
    {
        joint_pos_pub_ = n_.advertise<sensor_msgs::JointState>("target_joint_pos", 1);
		joint_vel_pub_ = n_.advertise<brics_actuator::JointVelocities>("command_vel", 1);
        controller_state_ = n_.subscribe("state", 1, &cob_trajectory_controller::state_callback, this);
	operation_mode_ = n_.subscribe("current_operationmode", 1, &cob_trajectory_controller::operationmode_callback, this);
		srvServer_Stop_ = n_.advertiseService("stop", &cob_trajectory_controller::srvCallback_Stop, this);
		srvClient_SetOperationMode = n_.serviceClient<cob_srvs::SetOperationMode>("set_operation_mode");
		while(!srvClient_SetOperationMode.exists())
		  {
		    ROS_INFO("Waiting for operationmode service to become available");
		      sleep(1);
		  }
        executing_ = false;
	watchdog_counter = 0;
	current_operation_mode_ = "undefined";
		q_current.resize(7);
		traj_generator_ = new genericArmCtrl(7);
		//TODO here set velocities and accelerations from parameter server
	}

	bool srvCallback_Stop(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
  	{
		ROS_INFO("Stopping powercubes...");

		// stop trajectory controller
		executing_ = false;
		res.success.data = true;
		ROS_INFO("...stopping cob_trajectory_controller.");
		traj_generator_->isMoving = false;
		//as_.setPreemted();
		return true;
  	}

  void operationmode_callback(const std_msgs::StringPtr& message)
  {
    current_operation_mode_ = message->data;
  }
  void state_callback(const pr2_controllers_msgs::JointTrajectoryControllerStatePtr& message)
  {
    std::vector<double> positions = message->actual.positions;
    for(unsigned int i = 0; i < positions.size(); i++)
    {
      q_current[i] = positions[i];
    }
  }
  void executeTrajectory(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr &goal)
  {
        ROS_INFO("Received new goal trajectory with %d points",goal->trajectory.points.size());
        if(!executing_)
        {
	  //set arm to velocity mode
	  cob_srvs::SetOperationMode opmode;
	  opmode.request.operation_mode.data = "velocity";
	  srvClient_SetOperationMode.call(opmode);
	  while(current_operation_mode_ != "velocity")
	    {
	      ROS_INFO("waiting for arm to go to velocity mode");
	      usleep(100000);
	    }
            traj_ = goal->trajectory;
            if(traj_.points.size() == 1)
            {
            	traj_generator_->moveThetas(traj_.points[0].positions, q_current);
            }
            else
            {
            	//Insert the current point as first point of trajectory, then generate SPLINE trajectory
            	trajectory_msgs::JointTrajectoryPoint p;
            	p.positions.resize(7);
            	p.velocities.resize(7);
            	p.accelerations.resize(7);
            	for(unsigned int i = 0; i<7; i++)
            	{
            		p.positions.at(i) = q_current.at(i);
            		p.velocities.at(i) = 0.0;
            		p.accelerations.at(i) = 0.0;
            	}
            	std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator it;
            	it = traj_.points.begin();
            	traj_.points.insert(it,p);
            	traj_generator_->moveTrajectory(traj_, q_current);
            }
            executing_ = true;
            startposition_ = q_current;

            //TODO: std::cout << "Trajectory time: " << traj_end_time_ << " Trajectory step: " << traj_step_ << "\n";
		    }
	    else //suspend current movement and start new one
	    {
	   
	    }
		while(executing_)
		{
			sleep(1);
		}
		as_.setSucceeded();
    }
    
    void run()
    {
        if(executing_)
        {
	  watchdog_counter = 0;
			if (as_.isPreemptRequested() || !ros::ok() || current_operation_mode_ != "velocity")
			{
				
				// set the action state to preempted
				executing_ = false;
				traj_generator_->isMoving = false;
				//as_.setPreempted();
				ROS_INFO("Preempted trajectory action");
				return;
			}
        	std::vector<double> des_vel;
        	if(traj_generator_->step(q_current, des_vel))
        	{
        		if(!traj_generator_->isMoving) //Finished trajectory
        		{
        			executing_ = false;
        		}
				brics_actuator::JointVelocities target_joint_vel;
				target_joint_vel.velocities.resize(7);
				for(unsigned int i=0; i<7; i++)
				{
					std::stringstream joint_name;
					joint_name << "arm_" << (i+1) << "_joint";
					target_joint_vel.velocities[i].joint_uri = joint_name.str();
					target_joint_vel.velocities[i].unit = "rad";
					target_joint_vel.velocities[i].value = des_vel.at(i);

				}

				//send everything
				joint_vel_pub_.publish(target_joint_vel);
        	}
        	else
        	{
        		ROS_INFO("An controller error occured!");
        		executing_ = false;
        	}
        }
		else
		{	//WATCHDOG TODO: don't always send
		  if(watchdog_counter < 10)
		    {
			sensor_msgs::JointState target_joint_position;
			target_joint_position.position.resize(7);
			brics_actuator::JointVelocities target_joint_vel;
			target_joint_vel.velocities.resize(7);
			for (unsigned int i = 0; i < 7; i += 1)
			{
				std::stringstream joint_name;
				joint_name << "arm_" << (i+1) << "_joint";
				target_joint_vel.velocities[i].joint_uri = joint_name.str();
						target_joint_position.position[i] = 0;
				target_joint_vel.velocities[i].unit = "rad";
				target_joint_vel.velocities[i].value = 0;
					}
				joint_vel_pub_.publish(target_joint_vel);
				joint_pos_pub_.publish(target_joint_position);
			}
		  watchdog_counter++;
		}
    }
    
};



int main(int argc, char ** argv)
{
    ros::init(argc, argv, "cob_trajectory_controller");
    cob_trajectory_controller tm;
    ros::Rate loop_rate(HZ);
    while (ros::ok())
    {
        tm.run();
        ros::spinOnce();
        loop_rate.sleep();
    }  
	
}






