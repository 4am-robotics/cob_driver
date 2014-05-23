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
 *   This class provides a twist_generator for tracking a given tf-frame
 *
 ****************************************************************/
#include <ros/ros.h>

#include <cob_lookat_controller/cob_frame_tracker.h>


void CobFrameTracker::initialize()
{
	///get params
	if (nh_.hasParam("update_rate"))
	{	nh_.getParam("update_rate", update_rate_);	}
	else
	{	update_rate_ = 10.0;	}	//hz
	
	if (nh_.hasParam("max_vel_lin"))
	{	nh_.getParam("max_vel_lin", max_vel_lin_);	}
	else
	{	max_vel_lin_ = 0.5;	}	//m/sec
	
	if (nh_.hasParam("max_vel_rot"))
	{	nh_.getParam("max_vel_rot", max_vel_rot_);	}
	else
	{	max_vel_rot_ = 0.3;	}	//rad/sec
	
	start_server_ = nh_.advertiseService("start_tracking", &CobFrameTracker::start_tracking_cb, this);
	stop_server_ = nh_.advertiseService("stop_tracking", &CobFrameTracker::stop_tracking_cb, this);
	twist_pub_ = nh_.advertise<geometry_msgs::Twist> ("command_twist", 1);
	
	tracking_frame_ = "/lookat_focus_frame";
	tracking_ = false;
	
	ROS_INFO("...initialized!");
}

void CobFrameTracker::run()
{
	ros::Rate r(update_rate_);
	while(ros::ok())
	{
		if(tracking_)
			publish_twist();
		
		ros::spinOnce();
		r.sleep();
	}
}

void CobFrameTracker::publish_twist()
{
	tf::StampedTransform transform_tf;
	geometry_msgs::TransformStamped transform_msg;
	geometry_msgs::Twist twist_msg;
	try{
		tf_listener_.lookupTransform("/lookat_focus_frame", tracking_frame_, ros::Time(0), transform_tf);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}
	
	tf::transformStampedTFToMsg(transform_tf, transform_msg);
	//twist_msg.linear.x = transform_msg.transform.translation.x/(max_vel_lin_/update_rate_);
	//twist_msg.linear.y = transform_msg.transform.translation.y/(max_vel_lin_/update_rate_);
	//twist_msg.linear.z = transform_msg.transform.translation.z/(max_vel_lin_/update_rate_);
	twist_msg.linear.x = copysign(std::min(max_vel_lin_, std::fabs(transform_msg.transform.translation.x)),transform_msg.transform.translation.x);
	twist_msg.linear.y = copysign(std::min(max_vel_lin_, std::fabs(transform_msg.transform.translation.y)),transform_msg.transform.translation.y);
	twist_msg.linear.z = copysign(std::min(max_vel_lin_, std::fabs(transform_msg.transform.translation.z)),transform_msg.transform.translation.z);
	twist_msg.angular.x = copysign(std::min(max_vel_rot_, std::fabs(transform_msg.transform.rotation.x)),transform_msg.transform.rotation.x);
	twist_msg.angular.y = copysign(std::min(max_vel_rot_, std::fabs(transform_msg.transform.rotation.y)),transform_msg.transform.rotation.y);
	twist_msg.angular.z = copysign(std::min(max_vel_rot_, std::fabs(transform_msg.transform.rotation.z)),transform_msg.transform.rotation.z);
	
	twist_pub_.publish(twist_msg);
}

bool CobFrameTracker::start_tracking_cb(cob_srvs::SetString::Request& request, cob_srvs::SetString::Response& response)
{
	tracking_frame_ = request.data;
	tracking_ = true;
	
	response.success = true;
	
	return true;
}

bool CobFrameTracker::stop_tracking_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	tracking_frame_ = "/lookat_focus_frame";
	tracking_ = false;
	
	//publish zero Twist for stopping
	geometry_msgs::Twist twist_msg;
	twist_pub_.publish(twist_msg);
	
	return true;
}

