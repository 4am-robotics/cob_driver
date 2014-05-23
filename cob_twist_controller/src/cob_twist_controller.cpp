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
	XmlRpc::XmlRpcValue jn_param;
	if (nh_.hasParam("joint_names"))
	{	nh_.getParam("joint_names", jn_param);	}
	else
	{	ROS_ERROR("Parameter joint_names not set");	}
	
	dof_ = jn_param.size();
	for(unsigned int i=0; i<dof_; i++)
	{	joints_.push_back((std::string)jn_param[i]);	}
	
	////used only for p_iksolver_pos_
	//limits_min_.push_back(-6.2831);
	//limits_min_.push_back(-6.2831);
	//limits_min_.push_back(-6.2831);
	//limits_max_.push_back(6.2831);
	//limits_max_.push_back(6.2831);
	//limits_max_.push_back(6.2831);
	
	//ToDo: get this from urdf
	double vel_param;
	if (nh_.hasParam("ptp_vel"))
	{
		nh_.getParam("ptp_vel", vel_param);
	}
	limits_vel_.push_back(vel_param);
	limits_vel_.push_back(vel_param);
	limits_vel_.push_back(vel_param);
	
	if (nh_.hasParam("base_link"))
	{
		nh_.getParam("base_link", chain_base_);
	}
	if (nh_.hasParam("tip_link"))
	{
		nh_.getParam("tip_link", chain_tip_);
	}
	
	
	
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
	
	
	////used only for p_iksolver_pos_
	//KDL::JntArray chain_min(limits_min_.size());
	//KDL::JntArray chain_max(limits_max_.size());
	//for(unsigned int i=0; i<limits_min_.size(); i++)
	//{
		//chain_min(i)=limits_min_[i];
		//chain_max(i)=limits_max_[i];
	//}
	
	
	///initialize configuration control solver
	//p_fksolver_pos_ = new KDL::ChainFkSolverPos_recursive(chain_);
	//p_fksolver_vel_ = new KDL::ChainFkSolverVel_recursive(chain_);
	
	p_iksolver_vel_ = new KDL::ChainIkSolverVel_pinv(chain_, 0.001, 5);
	p_iksolver_vel_wdls_ = new KDL::ChainIkSolverVel_wdls(chain_, 0.001, 5);
	//Eigen::MatrixXd Mq;
	//p_iksolver_vel_wdls_->setWeightJS(Mq);
	//Eigen::MatrixXd Mx;
	//p_iksolver_vel_wdls_->setWeightTS(Mx);
	double max_vel = vel_param;
	//double lambda = 1.0/(2*max_vel);
	double lambda = 100.0;
	p_iksolver_vel_wdls_->setLambda(lambda);
	
	//p_iksolver_pos_ = new KDL::ChainIkSolverPos_NR_JL(chain_, chain_min, chain_max, *p_fksolver_pos_, *p_iksolver_vel_, 50, 0.001);
	
	
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
	tf::StampedTransform transform_tf;
	KDL::Frame frame;
	try{
		tf_listener_.lookupTransform(chain_base_, chain_tip_, ros::Time(0), transform_tf);
		frame.p = KDL::Vector(transform_tf.getOrigin().x(), transform_tf.getOrigin().y(), transform_tf.getOrigin().z());
		frame.M = KDL::Rotation::Quaternion(transform_tf.getRotation().x(), transform_tf.getRotation().y(), transform_tf.getRotation().z(), transform_tf.getRotation().w());
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}
	
	KDL::Twist twist;
	tf::twistMsgToKDL(*msg, twist);
	KDL::JntArray q_dot_ik(chain_.getNrOfJoints());
	
	//ROS_INFO("Twist Vel (%f, %f, %f)", twist.vel.x(), twist.vel.y(), twist.vel.z());
	//ROS_INFO("Twist Rot (%f, %f, %f)", twist.rot.x(), twist.rot.y(), twist.rot.z());
	
	KDL::Twist twist_transformed = frame*twist;
	
	//ROS_INFO("TwistTransformed Vel (%f, %f, %f)", twist_transformed.vel.x(), twist_transformed.vel.y(), twist_transformed.vel.z());
	//ROS_INFO("TwistTransformed Rot (%f, %f, %f)", twist_transformed.rot.x(), twist_transformed.rot.y(), twist_transformed.rot.z());
	
	//std::cout << "Current q: ";
	//for(unsigned int i=0; i<last_q_.rows(); i++)
	//{
		//std::cout << last_q_(i) << ", ";
	//}
	//std::cout << std::endl;
	
	
	//int ret_ik = p_iksolver_vel_->CartToJnt(last_q_, twist_transformed, q_dot_ik);
	int ret_ik = p_iksolver_vel_wdls_->CartToJnt(last_q_, twist_transformed, q_dot_ik);
	
	if(ret_ik < 0)
	{
		ROS_ERROR("No Vel-IK found!");
	}
	else
	{
		//std::cout << "Solution q_dot: ";
		//for(unsigned int i=0; i<q_dot_ik.rows(); i++)
		//{
			//std::cout << q_dot_ik(i) << ", ";
		//}
		//std::cout << std::endl;
		
		brics_actuator::JointVelocities vel_msg;
		vel_msg.velocities.resize(joints_.size());
		for(int i=0; i<joints_.size(); i++)
		{
			vel_msg.velocities[i].joint_uri = joints_[i].c_str();
			vel_msg.velocities[i].unit = "rad";
			vel_msg.velocities[i].value = (std::fabs(q_dot_ik(i)) >= limits_vel_[i]) ? limits_vel_[i] : q_dot_ik(i);
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
		//ROS_DEBUG("Done Parsing");
		last_q_ = q_temp;
		last_q_dot_ = q_dot_temp;
	}
}
