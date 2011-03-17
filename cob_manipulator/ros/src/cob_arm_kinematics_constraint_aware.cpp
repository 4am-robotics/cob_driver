/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Sachin Chitta
 */

#include "cob_manipulator/cob_arm_kinematics_constraint_aware.h"

#include <pr2_arm_kinematics/pr2_arm_kinematics_utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <ros/ros.h>
#include <algorithm>
#include <numeric>

#include <sensor_msgs/JointState.h>
#include <kinematics_msgs/utils.h>
#include <visualization_msgs/Marker.h>


using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;



// initialize the constants for the service name and timeout
const std::string CobArmIKConstraintAware::IK_WITH_COLLISION_SERVICE = "/arm_controller/get_constraint_aware_ik";
const double CobArmIKConstraintAware::IK_DEFAULT_TIMEOUT = 10.0;


CobArmIKConstraintAware::CobArmIKConstraintAware() : node_handle_("~")
{
	dimension_ = 7;
	setup_collision_environment_ = false;
	



	// load the model of the robot from the parameter server so that it can be
	// used for collision checks
	urdf::Model robot_model;
	std::string tip_name;
	std::string xml_string;
	while (!pr2_arm_kinematics::loadRobotModel(node_handle_, robot_model, root_name_, tip_name, xml_string) && node_handle_.ok()) {
		ROS_ERROR("Could not load robot model. Are you sure the robot model is on the parameter server?");
		ros::Duration(0.5).sleep();
	}
	
	// get parameters from parameter server
	node_handle_.param<int>("free_angle", free_angle_, 2);
	node_handle_.param<double>("search_discretization", search_discretization_, 0.01);
	
	// create a new IK solver
	cob_arm_ik_solver_.reset(new CobArmIKSolver(robot_model, root_name_, tip_name, search_discretization_, free_angle_));
	
	// fetch information about the IK
	cob_arm_ik_solver_->getSolverInfo(ik_solver_info_);


	node_handle_.param<bool>("visualize_solution", visualize_solution_, true);
	ROS_DEBUG("Advertising services");

	ROS_INFO("Loading cob_arm_kinematics_constraint_aware server");
	advertiseIK();

	if(setupCollisionEnvironment()) ROS_INFO("Collision environment setup.");
	else ROS_ERROR("Could not initialize collision environment");
}


CobArmIKConstraintAware::~CobArmIKConstraintAware()
{
	if (planning_monitor_) delete planning_monitor_;
	if (collision_models_) delete collision_models_;
}


void CobArmIKConstraintAware::advertiseIK()
{
	ik_collision_service_ = node_handle_.advertiseService(IK_WITH_COLLISION_SERVICE, &CobArmIKConstraintAware::getConstraintAwarePositionIK, this);
	display_trajectory_publisher_ = root_handle_.advertise<motion_planning_msgs::DisplayTrajectory>("ik_solution_display", 1);
}


bool CobArmIKConstraintAware::isReady(motion_planning_msgs::ArmNavigationErrorCodes &error_code)
{
	if (!setup_collision_environment_) {
		ROS_INFO("Waiting for collision environment setup.");
		if (!setupCollisionEnvironment()) {
			ROS_INFO("Could not initialize collision environment");
			error_code.val = error_code.COLLISION_CHECKING_UNAVAILABLE;
			
			return false;
		} else {
			setup_collision_environment_ = true;
		}
	}
	error_code.val = error_code.SUCCESS;
	
	return true;
}


int CobArmIKConstraintAware::CartToJntSearch(const KDL::JntArray& q_in, 
												const KDL::Frame& p_in, 
												KDL::JntArray &q_out, 
												const double &timeout, 
												motion_planning_msgs::ArmNavigationErrorCodes& error_code)
{
	if (!isReady(error_code)) return -1;
	
	bool ik_valid = (cob_arm_ik_solver_->CartToJntSearch(q_in,
							p_in,
							q_out, 
							timeout,
							error_code,
							boost::bind(&CobArmIKConstraintAware::initialPoseCheck, this, _1, _2, _3),
							boost::bind(&CobArmIKConstraintAware::collisionCheck, this, _1, _2, _3)) >= 0);

	return ik_valid;
}


bool CobArmIKConstraintAware::getConstraintAwarePositionIK(kinematics_msgs::GetConstraintAwarePositionIK::Request &request_in,
															kinematics_msgs::GetConstraintAwarePositionIK::Response &response)
{
	if (!isReady(response.error_code)) return true;

	if (!pr2_arm_kinematics::checkConstraintAwareIKService(request_in, response, ik_solver_info_)) {
		ROS_ERROR("IK service request is malformed");
		
		return true;
	}

	ros::Time start_time = ros::Time::now();
	ROS_INFO("Received IK request is in the frame: %s", request_in.ik_request.pose_stamped.header.frame_id.c_str());

	ik_request_ = request_in.ik_request;
	collision_operations_ = request_in.ordered_collision_operations;
	link_padding_ = request_in.link_padding;
	allowed_contacts_ = request_in.allowed_contacts;
	constraints_ = request_in.constraints;

	geometry_msgs::PoseStamped pose_msg_in = ik_request_.pose_stamped;
	geometry_msgs::PoseStamped pose_msg_out;
	
	if(!pr2_arm_kinematics::convertPoseToRootFrame(pose_msg_in, pose_msg_out, root_name_, tf_)) {
		response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
		
		return true;
	}


	ik_request_.pose_stamped = pose_msg_out;
	ROS_INFO("Transformed IK request is in the frame: %s", ik_request_.pose_stamped.header.frame_id.c_str());

	KDL::JntArray jnt_pos_out;
	KDL::JntArray jnt_pos_in;
	KDL::Frame p_in;
	tf::PoseMsgToKDL(pose_msg_out.pose, p_in);
	jnt_pos_in.resize(dimension_);
	jnt_pos_out.resize(dimension_);
	
	for (int i = 0; i < dimension_; i++) {
		int tmp_index = pr2_arm_kinematics::getJointIndex(request_in.ik_request.ik_seed_state.joint_state.name[i], ik_solver_info_);
		jnt_pos_in(tmp_index) = request_in.ik_request.ik_seed_state.joint_state.position[i];
	}
	
	ros::Time ik_solver_time = ros::Time::now();
	bool ik_valid = CartToJntSearch(jnt_pos_in,
									p_in,
									jnt_pos_out, 
									request_in.timeout.toSec(),
									response.error_code);
	ROS_INFO("IK solver time: %f", (ros::Time::now() - ik_solver_time).toSec());
	
	if (ik_valid) {
		response.solution.joint_state.name = ik_solver_info_.joint_names;
		response.solution.joint_state.position.resize(dimension_);
		
		for (int i = 0; i < dimension_; i++) {
			response.solution.joint_state.position[i] = jnt_pos_out(i);
			
			ROS_DEBUG("IK Solution: %s %d: %f",response.solution.joint_state.name[i].c_str(), i, jnt_pos_out(i));
		}
		
		if(visualize_solution_) {
			motion_planning_msgs::DisplayTrajectory display_trajectory;
			display_trajectory.trajectory.joint_trajectory.points.resize(1);
			display_trajectory.trajectory.joint_trajectory.points[0].positions = response.solution.joint_state.position;
			display_trajectory.trajectory.joint_trajectory.joint_names = response.solution.joint_state.name;
			planning_monitor_->getRobotStateMsg(display_trajectory.robot_state);
			display_trajectory_publisher_.publish(display_trajectory);
		}
		
		ROS_INFO("IK service time: %f",(ros::Time::now() - start_time).toSec());
		response.error_code.val = response.error_code.SUCCESS;
		
		return true;
	} else {
		ROS_ERROR("An IK solution could not be found");   
		
		return true;
	}
}


void CobArmIKConstraintAware::collisionCheck(const KDL::JntArray &jnt_array, 
                                           const KDL::Frame &p_in,
                                           motion_planning_msgs::ArmNavigationErrorCodes &error_code)
{
  planning_monitor_->getEnvironmentModel()->lock();
  planning_monitor_->getKinematicModel()->lock();
  sensor_msgs::JointState joint_state = ik_request_.robot_state.joint_state;
  for(unsigned int i=0; i < ik_solver_info_.joint_names.size(); i++)
  {
    joint_state.position.push_back(jnt_array(i));
    joint_state.name.push_back(ik_solver_info_.joint_names[i]);
  }
  planning_monitor_->applyLinkPaddingToCollisionSpace(link_padding_);
  planning_monitor_->setJointStateAndComputeTransforms(joint_state);
  planning_monitor_->getEnvironmentModel()->updateRobotModel();

  motion_planning_msgs::OrderedCollisionOperations operations;
  std::vector<std::string> child_links;
  planning_monitor_->getChildLinks(joint_state.name, child_links);
  planning_monitor_->getOrderedCollisionOperationsForOnlyCollideLinks(child_links,collision_operations_,operations);
  planning_monitor_->applyOrderedCollisionOperationsToCollisionSpace(operations);
  planning_monitor_->setAllowedContacts(allowed_contacts_);

  bool check = (!planning_monitor_->getEnvironmentModel()->isCollision() &&  !planning_monitor_->getEnvironmentModel()->isSelfCollision());
  if(!check) {
    planning_monitor_->broadcastCollisions();
    error_code.val = error_code.KINEMATICS_STATE_IN_COLLISION;
  }
  else
    error_code.val = error_code.SUCCESS;

  if(!planning_monitor_->checkConstraintsAtCurrentState(constraints_,error_code))
    ROS_DEBUG("Constraints violated at current state");

  ROS_DEBUG("Solution collision check done with result %s",check ? "not_in_collision" : "in_collision" );
  planning_monitor_->revertAllowedCollisionToDefault();
  planning_monitor_->clearAllowedContacts();
  planning_monitor_->revertCollisionSpacePaddingToDefault();
  planning_monitor_->getKinematicModel()->unlock();
  planning_monitor_->getEnvironmentModel()->unlock();
}


void CobArmIKConstraintAware::initialPoseCheck(const KDL::JntArray &jnt_array, 
                                             const KDL::Frame &p_in,
                                             motion_planning_msgs::ArmNavigationErrorCodes &error_code)
{
  std::string kinematic_frame_id = cob_arm_ik_solver_->getFrameId();
  std::string planning_frame_id = planning_monitor_->getFrameId();
  geometry_msgs::PoseStamped pose_stamped;
  tf::PoseKDLToMsg(p_in,pose_stamped.pose);
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.frame_id = kinematic_frame_id;
  if (!tf_.canTransform(planning_frame_id,
                        pose_stamped.header.frame_id,
                        pose_stamped.header.stamp))
  {
    std::string err;
    ros::Time tmp;
    if(tf_.getLatestCommonTime(pose_stamped.header.frame_id,planning_frame_id,tmp,&err) != tf::NO_ERROR)
    {
      ROS_ERROR("Cannot transform from '%s' to '%s'. TF said: %s",
                pose_stamped.header.frame_id.c_str(),planning_frame_id.c_str(), err.c_str());
    }
    else
      pose_stamped.header.stamp = tmp;
  }
    
  try
  {
    tf_.transformPose(planning_frame_id,pose_stamped,pose_stamped);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("Cannot transform from '%s' to '%s'. Tf said: %s",
              pose_stamped.header.frame_id.c_str(),planning_frame_id.c_str(), ex.what());
    error_code.val = error_code.FRAME_TRANSFORM_FAILURE;
    return;
  }

  btTransform transform;
  tf::poseMsgToTF(pose_stamped.pose,transform);
  planning_models::KinematicModel::Link* end_effector_link = planning_monitor_->getKinematicModel()->getLink(ik_request_.ik_link_name);
  if(!end_effector_link)
  {
    ROS_ERROR("Could not find end effector root_link %s", ik_request_.ik_link_name.c_str());
    error_code.val = error_code.INVALID_LINK_NAME;
    return;
  }

  planning_monitor_->getEnvironmentModel()->lock();
  planning_monitor_->getKinematicModel()->lock();
  planning_monitor_->applyLinkPaddingToCollisionSpace(link_padding_);
  planning_monitor_->setJointStateAndComputeTransforms(ik_request_.robot_state.joint_state);
  end_effector_link->setTransform(transform);
  for(unsigned int i=0; i < end_effector_link->after.size(); i++)
    end_effector_link->after[i]->after->updateTransformsRecursive();
  std::vector<std::string> collision_check_links;
  std::vector<std::string> dummy_links;
  planning_monitor_->getEnvironmentModel()->updateRobotModel();

  motion_planning_msgs::OrderedCollisionOperations operations;
  std::vector<std::string> child_links;
  planning_monitor_->getOrderedCollisionOperationsForOnlyCollideLinks(end_effector_collision_links_,
                                                                      collision_operations_,
                                                                      operations);
  planning_monitor_->applyOrderedCollisionOperationsToCollisionSpace(operations);
  planning_monitor_->setAllowedContacts(allowed_contacts_);

  bool check = ( !planning_monitor_->getEnvironmentModel()->isCollision() &&  
            !planning_monitor_->getEnvironmentModel()->isSelfCollision() );
  if(!check) {
    error_code.val = error_code.IK_LINK_IN_COLLISION;
    planning_monitor_->broadcastCollisions();
  }
  else
    error_code.val = error_code.SUCCESS;
    
  planning_monitor_->clearConstraints();
  planning_monitor_->revertAllowedCollisionToDefault();
  planning_monitor_->clearAllowedContacts();
  planning_monitor_->revertCollisionSpacePaddingToDefault();
  planning_monitor_->getKinematicModel()->unlock();
  planning_monitor_->getEnvironmentModel()->unlock();
  ROS_DEBUG("Initial pose check done with result %s",check ? "not_in_collision" : "in_collision" );
}


void CobArmIKConstraintAware::printStringVec(const std::string &prefix, const std::vector<std::string> &string_vector)
{
  ROS_DEBUG("%s",prefix.c_str());
  for(unsigned int i=0; i < string_vector.size(); i++)
  {
    ROS_DEBUG("%s",string_vector[i].c_str());
  }
}


bool CobArmIKConstraintAware::setupCollisionEnvironment()
{
  node_handle_.param<std::string>("group", group_, std::string());
  node_handle_.param<bool>("use_collision_map", use_collision_map_, true);
  if (group_.empty())
  {
    ROS_ERROR("No 'group' parameter specified. Without the name of the group of joints to monitor, action cannot start");
    return false;
  }

  // monitor robot
  collision_models_ = new planning_environment::CollisionModels("robot_description");

  if(!collision_models_)
  {
    ROS_INFO("Could not initialize collision models");
    return false;
  }
  planning_monitor_ = new planning_environment::PlanningMonitor(collision_models_, &tf_);

  if(!planning_monitor_)
  {
    ROS_ERROR("Could not initialize planning monitor");
    return false;
  }
  else
  {
    ROS_INFO("Initialized planning monitor");
  }

  planning_monitor_->use_collision_map_ = use_collision_map_;
  if(!collision_models_->loadedModels())
  {
    ROS_ERROR("Could not load models");
    return false;
  }
  if (!collision_models_->getKinematicModel()->hasGroup(group_))
  {
    ROS_ERROR("Group '%s' is not known", group_.c_str());
    collision_models_->getKinematicModel()->printModelInfo(std::cout);
    return false;
  }
  else
    ROS_INFO("Configuring action for '%s'", group_.c_str());

  std::vector<planning_models::KinematicModel::Joint*> p_joints = collision_models_->getKinematicModel()->getGroup(group_)->joints;
  for(unsigned int i=0; i < p_joints.size(); i++)
  {
    default_collision_links_.push_back(p_joints[i]->after->name); 
  }

  if (planning_monitor_->getExpectedJointStateUpdateInterval() > 1e-3)
    planning_monitor_->waitForState();
  if (planning_monitor_->getExpectedMapUpdateInterval() > 1e-3 && use_collision_map_)
    planning_monitor_->waitForMap();
  //planning_monitor_->setCollisionCheckOnlyLinks(default_collision_links_,true);

  end_effector_collision_links_.clear();
  planning_models::KinematicModel::Link* end_effector_link = planning_monitor_->getKinematicModel()->getLink(ik_solver_info_.link_names.back());
  end_effector_link->getAllChildLinksRecursive(end_effector_collision_links_);
  for(unsigned int i=0; i < end_effector_collision_links_.size(); i++)
  {
    default_collision_links_.push_back(end_effector_collision_links_[i]);
  }

  printStringVec("Default collision links",default_collision_links_);
  printStringVec("End effector links",end_effector_collision_links_);

  ROS_DEBUG("Root link name is: %s",root_name_.c_str());

  planning_monitor_->setOnCollisionContactCallback(boost::bind(&CobArmIKConstraintAware::contactFound, this, _1));
  vis_marker_publisher_ = root_handle_.advertise<visualization_msgs::Marker>("kinematics_collisions", 128);

  setup_collision_environment_ = true;
  return true;
}


void CobArmIKConstraintAware::contactFound(collision_space::EnvironmentModel::Contact &contact)
{

  static int count = 0;
  
  std::string ns_name;
  if(contact.link1 != NULL) {
    //ROS_INFO_STREAM("Link 1 is " << contact.link2->name);
    if(contact.link1_attached_body_index == 0) {
      ns_name += contact.link1->name+"+";
    } else {
      if(contact.link1->attachedBodies.size() < contact.link1_attached_body_index) {
        ROS_ERROR("Link doesn't have attached body with indicated index");
      } else {
        ns_name += contact.link1->attachedBodies[contact.link1_attached_body_index-1]->id+"+";
      }
    }
  } 
  
  if(contact.link2 != NULL) {
    //ROS_INFO_STREAM("Link 2 is " << contact.link2->name);
    if(contact.link2_attached_body_index == 0) {
      ns_name += contact.link2->name;
    } else {
      if(contact.link2->attachedBodies.size() < contact.link2_attached_body_index) {
        ROS_ERROR("Link doesn't have attached body with indicated index");
      } else {
        ns_name += contact.link2->attachedBodies[contact.link2_attached_body_index-1]->id;
      }
    }
  } 
  
  if(!contact.object_name.empty()) {
    //ROS_INFO_STREAM("Object is " << contact.object_name);
    ns_name += contact.object_name;
  }
  
  visualization_msgs::Marker mk;
  mk.header.stamp = planning_monitor_->lastJointStateUpdate();
  mk.header.frame_id = planning_monitor_->getFrameId();
  mk.ns = ns_name;
  mk.id = count++;
  mk.type = visualization_msgs::Marker::SPHERE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.position.x = contact.pos.x();
  mk.pose.position.y = contact.pos.y();
  mk.pose.position.z = contact.pos.z();
  mk.pose.orientation.w = 1.0;
  
  mk.scale.x = mk.scale.y = mk.scale.z = 0.01;
  
  mk.color.a = 0.6;
  mk.color.r = 1.0;
  mk.color.g = 0.04;
  mk.color.b = 0.04;
  
  //mk.lifetime = ros::Duration(30.0);
  
  vis_marker_publisher_.publish(mk);
}

