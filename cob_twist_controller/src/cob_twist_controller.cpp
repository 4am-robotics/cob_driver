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
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Felix Messmer, email: Felix.Messmer@ipa.fraunhofer.de
 *
 * \date Date of creation: April, 2014
 *
 * \brief
 *   This package provides a generic Twist controller for the Care-O-bot
 *
 ****************************************************************/
#include <ros/ros.h>

#include <cob_twist_controller/cob_twist_controller.h>

#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>


void CobTwistController::initialize()
{
	///get params
	//hardcoded for now -- get this from URDF or yaml file later
	joints_.push_back("torso_1_joint");
	joints_.push_back("torso_2_joint");
	joints_.push_back("torso_3_joint");
	
	limits_min_.push_back(-6.2831);
	limits_min_.push_back(-6.2831);
	limits_min_.push_back(-6.2831);
	limits_max_.push_back(6.2831);
	limits_max_.push_back(6.2831);
	limits_max_.push_back(6.2831);
	
	chain_base_ = "torso_base_link";
	chain_tip_ = "torso_center_link";
	
	
	
	///parse robot_description and generate KDL chains
	KDL::Tree my_tree;
	std::string robot_desc_string;
	nh_.param("/robot_description", robot_desc_string, std::string());
	if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return;
	}
	my_tree.getChain(chain_base_, chain_tip_, chain_);
	if(chain_.getNrOfJoints() == 0)
	{
		ROS_ERROR("Failed to initialize kinematic chain");
		return;
	}
	
	KDL::JntArray chain_min(limits_min_.size());
	KDL::JntArray chain_max(limits_max_.size());
	for(unsigned int i=0; i<limits_min_.size(); i++)
	{
		chain_min(i)=limits_min_[i];
		chain_max(i)=limits_max_[i];
	}
	
	
	///initialize configuration control solver
	p_fksolver_pos_ = new KDL::ChainFkSolverPos_recursive(chain_);
	p_fksolver_vel_ = new KDL::ChainFkSolverVel_recursive(chain_);
	p_iksolver_vel_ = new KDL::ChainIkSolverVel_pinv(chain_, 0.001, 5);
	p_iksolver_pos_ = new KDL::ChainIkSolverPos_NR_JL(chain_, chain_min, chain_max, *p_fksolver_pos_, *p_iksolver_vel_, 50, 0.001);
	
	
	///initialize ROS interfaces
	jointstate_sub = nh_.subscribe("/joint_states", 1, &CobTwistController::jointstate_cb, this);
	twist_sub = nh_.subscribe("command_twist", 1, &CobTwistController::twist_cb, this);
	vel_pub = nh_.advertise<brics_actuator::JointVelocities>("command_vel", 10);
	
	
	///initialize variables and current joint values and velocities
	last_q_ = KDL::JntArray(chain_.getNrOfJoints());
	last_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());
	
	ROS_INFO("...initialized!");
}

void CobTwistController::run()
{
	ROS_INFO("cob_twist_controller...spinning");
	ros::spin();
}


void CobTwistController::twist_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
	KDL::Twist twist;
	tf::twistMsgToKDL(*msg, twist);
	KDL::JntArray q_dot_ik(chain_.getNrOfJoints());
	
	ROS_INFO("Twist Vel (%f, %f, %f)", twist.vel.x(), twist.vel.y(), twist.vel.z());
	ROS_INFO("Twist Rot (%f, %f, %f)", twist.rot.x(), twist.rot.y(), twist.rot.z());
	
	std::cout << "Current q : ";
	for(unsigned int i=0; i<last_q_.rows(); i++)
	{
		std::cout << last_q_(i) << ", ";
	}
	std::cout << std::endl;
	
	
	int ret_ik = p_iksolver_vel_->CartToJnt(last_q_, twist, q_dot_ik);
	
	if(ret_ik < 0)
	{
		ROS_ERROR("No Vel-IK found!");
	}
	else
	{
		brics_actuator::JointVelocities vel_msg;
		vel_msg.velocities.resize(joints_.size());
		for(int i=0; i<joints_.size(); i++)
		{
			vel_msg.velocities[i].joint_uri = joints_[i].c_str();
			vel_msg.velocities[i].unit = "rad";
			vel_msg.velocities[i].value = q_dot_ik(i);
		}
		vel_pub.publish(vel_msg);
	}
}


void CobTwistController::jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
	KDL::JntArray q_temp = last_q_;
	KDL::JntArray q_dot_temp = last_q_dot_;
	int count = 0;
	
	for(unsigned int j = 0; j < joints_.size(); j++)
	{
		for(unsigned int i = 0; i < msg->name.size(); i++)
		{
			if(strcmp(msg->name[i].c_str(), joints_[j].c_str()) == 0)
			{
				q_temp(j) = msg->position[i];
				q_dot_temp(j) = msg->velocity[i];
				count++;
				break;
			}
		}
	}
	
	if(count == joints_.size())
	{
		ROS_DEBUG("Done Parsing");
		last_q_ = q_temp;
		last_q_dot_ = q_dot_temp;
	}
}
