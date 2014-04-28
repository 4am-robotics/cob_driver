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
#include <ros/ros.h>

#include <cob_lookat_controller/cob_lookat_controller.h>

#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>


void CobLookatController::initialize()
{
	///get params
	XmlRpc::XmlRpcValue lookat_jn_param;
	if (nh_.hasParam("lookat_joint_names"))
	{	nh_.getParam("lookat_joint_names", lookat_jn_param);	}
	else
	{	ROS_ERROR("Parameter lookat_joint_names not set");	}
	
	lookat_dof_ = lookat_jn_param.size();
	for(unsigned int i=0; i<lookat_dof_; i++)
	{	lookat_joints_.push_back((std::string)lookat_jn_param[i]);	}
	
	XmlRpc::XmlRpcValue chain_jn_param;
	if (nh_.hasParam("chain_joint_names"))
	{	nh_.getParam("chain_joint_names", chain_jn_param);	}
	else
	{	ROS_ERROR("Parameter chain_joint_names not set");	}
	
	chain_dof_ = chain_jn_param.size();
	for(unsigned int i=0; i<chain_dof_; i++)
	{	chain_joints_.push_back((std::string)chain_jn_param[i]);	}
	
	total_dof_ = chain_dof_ + lookat_dof_;
	total_joints_ = chain_joints_;
	total_joints_.insert(total_joints_.end(), lookat_joints_.begin(), lookat_joints_.end());
	
	//ToDo: get this from urdf
	double vel_param;
	if (nh_.hasParam("ptp_vel"))
	{
		nh_.getParam("ptp_vel", vel_param);
	}
	chain_limits_vel_.assign(chain_dof_, vel_param);
	
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
	
	
	///initialize configuration control solver
	p_iksolver_vel_ = new KDL::ChainIkSolverVel_pinv(chain_, 0.001, 5);
	p_iksolver_vel_wdls_ = new KDL::ChainIkSolverVel_wdls(chain_, 0.001, 5);
	//Eigen::MatrixXd Mq;
	//p_iksolver_vel_wdls_->setWeightJS(Mq);
	//Eigen::MatrixXd Mx;
	//p_iksolver_vel_wdls_->setWeightTS(Mx);
	
	
	if (nh_.hasParam("chain_vel_pub_topic"))
	{	nh_.getParam("chain_vel_pub_topic", chain_vel_pub_topic_);	}
	else
	{	ROS_ERROR("Parameter chain_vel_pub_topic not set");	}
	
	///initialize ROS interfaces
	jointstate_sub = nh_.subscribe("/joint_states", 10, &CobLookatController::jointstate_cb, this);
	//lookatstate_sub = nh_.subscribe("/lookat_controller/joint_states", 1, &CobLookatController::lookatstate_cb, this);
	twist_sub = nh_.subscribe("command_twist", 1, &CobLookatController::twist_cb, this);
	chain_vel_pub = nh_.advertise<brics_actuator::JointVelocities>(chain_vel_pub_topic_, 10);
	lookat_vel_pub = nh_.advertise<brics_actuator::JointVelocities>("lookat_command_vel", 10);
	
	
	///initialize variables and current joint values and velocities
	last_q_ = KDL::JntArray(chain_.getNrOfJoints());
	last_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());
	
	ROS_INFO("...initialized!");
}

void CobLookatController::run()
{
	ROS_INFO("cob_lookat_controller...spinning");
	ros::spin();
}


void CobLookatController::twist_cb(const geometry_msgs::Twist::ConstPtr& msg)
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
	
	
	///ToDo: Verify this transformation
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
		
		brics_actuator::JointVelocities chain_vel_msg;
		chain_vel_msg.velocities.resize(chain_dof_);
		for(int i=0; i<chain_dof_; i++)
		{
			chain_vel_msg.velocities[i].joint_uri = chain_joints_[i].c_str();
			chain_vel_msg.velocities[i].unit = "rad";
			chain_vel_msg.velocities[i].value = (std::fabs(q_dot_ik(i)) >= chain_limits_vel_[i]) ? chain_limits_vel_[i] : q_dot_ik(i);
		}
		brics_actuator::JointVelocities lookat_vel_msg;
		lookat_vel_msg.velocities.resize(lookat_dof_);
		for(int i=0; i<lookat_dof_; i++)
		{
			lookat_vel_msg.velocities[i].joint_uri = lookat_joints_[i].c_str();
			lookat_vel_msg.velocities[i].unit = "rad";
			lookat_vel_msg.velocities[i].value = (q_dot_ik(chain_dof_ + i) >= 0.2) ? 0.2 : q_dot_ik(chain_dof_ + i);
		}
		
		
		chain_vel_pub.publish(chain_vel_msg);
		lookat_vel_pub.publish(lookat_vel_msg);
	}
}


void CobLookatController::jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
	KDL::JntArray q_temp = last_q_;
	KDL::JntArray q_dot_temp = last_q_dot_;
	int chain_count = 0;
	int lookat_count = 0;
	
	for(unsigned int j = 0; j < chain_dof_; j++)
	{
		for(unsigned int i = 0; i < msg->name.size(); i++)
		{
			if(strcmp(msg->name[i].c_str(), chain_joints_[j].c_str()) == 0)
			{
				q_temp(j) = msg->position[i];
				q_dot_temp(j) = msg->velocity[i];
				chain_count++;
				break;
			}
		}
	}
	for(unsigned int j = 0; j < lookat_dof_; j++)
	{
		for(unsigned int i = 0; i < msg->name.size(); i++)
		{
			if(strcmp(msg->name[i].c_str(), lookat_joints_[j].c_str()) == 0)
			{
				q_temp(chain_dof_ + j) = msg->position[i];
				q_dot_temp(chain_dof_ + j) = msg->velocity[i];
				lookat_count++;
				break;
			}
		}
	}
	
	if(chain_count == chain_dof_ || lookat_count == lookat_dof_)
	{
		ROS_DEBUG("Done Parsing");
		last_q_ = q_temp;
		last_q_dot_ = q_dot_temp;
	}
}

void CobLookatController::lookatstate_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
	KDL::JntArray q_temp = last_q_;
	KDL::JntArray q_dot_temp = last_q_dot_;
	int count = 0;
	
	for(unsigned int j = 0; j < lookat_dof_; j++)
	{
		for(unsigned int i = 0; i < msg->name.size(); i++)
		{
			if(strcmp(msg->name[i].c_str(), lookat_joints_[j].c_str()) == 0)
			{
				q_temp(chain_dof_ + j) = msg->position[i];
				q_dot_temp(chain_dof_ + j) = msg->velocity[i];
				count++;
				break;
			}
		}
	}
	
	if(count == lookat_dof_)
	{
		ROS_DEBUG("Done Parsing");
		last_q_ = q_temp;
		last_q_dot_ = q_dot_temp;
	}
}
