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
#include <ros/ros.h>

#include <cob_lookat_controller/cob_lookat_driver.h>


void CobLookatDriver::initialize()
{
	///get params
	XmlRpc::XmlRpcValue jn_param;
	if (nh_.hasParam("lookat_joint_names"))
	{	nh_.getParam("lookat_joint_names", jn_param);	}
	else
	{	ROS_ERROR("Parameter lookat_joint_names not set");	}
	
	dof_ = jn_param.size();
	for(unsigned int i=0; i<dof_; i++)
	{	joints_.push_back((std::string)jn_param[i]);	}
	
	current_pos_.resize(dof_, 0.0);
	current_pos_[0] = 1.0;
	current_vel_.resize(dof_, 0.0);
	
	if (nh_.hasParam("update_rate"))
	{	nh_.getParam("update_rate", update_rate_);	}
	else
	{	update_rate_ = 100.0;	}
	
	command_vel_sub_ = nh_.subscribe("command_vel", 1, &CobLookatDriver::command_vel_cb, this);
	jointstate_pub_ = nh_.advertise<sensor_msgs::JointState> ("joint_states", 1);
	
	ROS_INFO("...initialized!");
}

void CobLookatDriver::run()
{
	ros::Rate r(update_rate_);
	while(ros::ok())
	{
		update_state();
		publish_state();
		
		ros::spinOnce();
		r.sleep();
	}
}

void CobLookatDriver::update_state()
{
	double dt = (ros::Time::now() - last_update_).toSec();
	
	for(unsigned int i=0; i<dof_; i++)
		current_pos_[i] += current_vel_[i]*dt;
}



void CobLookatDriver::publish_state()
{
	sensor_msgs::JointState msg;
	msg.header.stamp = ros::Time::now();
	for(unsigned int i=0; i<dof_; i++)
	{
		msg.name.push_back(joints_[i]);
		msg.position.push_back(current_pos_[i]);
		msg.velocity.push_back(current_vel_[i]);
		msg.effort.push_back(0.0);
	}
	
	jointstate_pub_.publish(msg);
}

void CobLookatDriver::command_vel_cb(const brics_actuator::JointVelocities::ConstPtr& msg)
{
	///ToDo: Do some checks!
	if(msg->velocities.size() != dof_)
	{
		ROS_ERROR("DoF do not match! Stopping!");
		current_vel_.assign(dof_, 0.0);
		return;
	}
	
	for(unsigned int i=0; i<dof_; i++)
	{
		current_vel_[i]=msg->velocities[i].value;
	}
	
	last_update_ = ros::Time::now();
}

