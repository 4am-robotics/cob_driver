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
 *   This class provides a controller for Lookat for the Care-O-bot (based on cob_twist_controller)
 *
 ****************************************************************/
#ifndef COB_LOOKAT_CONTROLLER_H
#define COB_LOOKAT_CONTROLLER_H

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <brics_actuator/JointVelocities.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>

#include <tf/transform_listener.h>


class CobLookatController
{
private:
	ros::NodeHandle nh_;
	tf::TransformListener tf_listener_;
	
	ros::Subscriber jointstate_sub;
	ros::Subscriber lookatstate_sub;
	ros::Subscriber twist_sub;
	ros::Publisher chain_vel_pub;
	ros::Publisher lookat_vel_pub;
	std::string chain_vel_pub_topic_;
	
	KDL::Chain chain_;
	std::string chain_base_;
	std::string chain_tip_;
	
	KDL::ChainIkSolverVel_pinv* p_iksolver_vel_;
	KDL::ChainIkSolverVel_wdls* p_iksolver_vel_wdls_;
	
	std::vector<std::string> chain_joints_;
	std::vector<std::string> lookat_joints_;
	std::vector<std::string> total_joints_;
	unsigned int chain_dof_;
	unsigned int lookat_dof_;
	unsigned int total_dof_;
	std::vector<float> chain_limits_vel_;
	
	KDL::JntArray last_q_;
	KDL::JntArray last_q_dot_;
	
	
public:
	CobLookatController() {;}
	~CobLookatController();
	
	void initialize();
	void run();
	
	void jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg);
	void lookatstate_cb(const sensor_msgs::JointState::ConstPtr& msg);
	void twist_cb(const geometry_msgs::Twist::ConstPtr& msg);
};

#endif

