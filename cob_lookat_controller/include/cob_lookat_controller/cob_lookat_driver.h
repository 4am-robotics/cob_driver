/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2014 \n
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
 *   ROS package name: cob_lookat_controller
 *
 * \author
 *   Author: Felix Messmer, email: Felix.Messmer@ipa.fraunhofer.de
 *
 * \date Date of creation: April, 2014
 *
 * \brief
 *   This class provides a virtual driver for the lookat kinematic chain
 *
 ****************************************************************/
#ifndef COB_LOOKAT_DRIVER_H
#define COB_LOOKAT_DRIVER_H

#include <ros/ros.h>

#include <brics_actuator/JointVelocities.h>
#include <sensor_msgs/JointState.h>

class CobLookatDriver
{
public:
	CobLookatDriver() {;}
	~CobLookatDriver();
	
	void initialize();
	void run();
	
	void update_state();
	void publish_state();
	void command_vel_cb(const brics_actuator::JointVelocities::ConstPtr& msg);
	
	
	ros::NodeHandle nh_;
	
	
	double update_rate_;
	ros::Time last_update_;
	
	unsigned int dof_;
	std::vector<std::string> joints_;
	
	std::vector<double> current_pos_;
	std::vector<double> current_vel_;
	
	ros::Subscriber command_vel_sub_;
	ros::Publisher jointstate_pub_;
};

#endif

