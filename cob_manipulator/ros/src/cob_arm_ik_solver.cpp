//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#include <cob_manipulator/cob_arm_ik_solver.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

using namespace Eigen;
using namespace pr2_arm_kinematics;


const int CobArmIKSolver::NO_IK_SOLUTION = -1;
const int CobArmIKSolver::TIMED_OUT = -2;


CobArmIKSolver::CobArmIKSolver(const urdf::Model &robot_model, 
								const std::string &root_frame_name,
								const std::string &tip_frame_name,
								const double &search_discretization_angle, 
								const int &free_angle)
{
	search_discretization_angle_ = search_discretization_angle;
	free_angle_ = free_angle;
	root_frame_name_ = root_frame_name;
	
	
	// get the information about the IK solver
	ros::ServiceClient ik_solver_info_client_;
	ik_solver_info_client_ = node_handle_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("/arm_controller/get_ik_solver_info");
	kinematics_msgs::GetKinematicSolverInfo::Request req;
	kinematics_msgs::GetKinematicSolverInfo::Response res;
	
	ros::service::waitForService("/arm_controller/get_ik_solver_info");
	ik_solver_info_client_.call(req, res);
	
	solver_info_ = res.kinematic_solver_info;
	

	// create the KDL chain from the description on the parameter server	
	KDL::Tree my_tree;
	ros::NodeHandle node;
	std::string robot_desc_string;
	node.param("/robot_description", robot_desc_string, std::string());
	if (!kdl_parser::treeFromString(robot_desc_string, my_tree)) {
		ROS_ERROR("Failed to construct kdl tree");
	}
	my_tree.getChain("arm_0_link", "arm_7_link", chain_);
	
	number_of_joints_ = chain_.getNrOfJoints();
	
	
	// setup the joint limits of the arm
	if (number_of_joints_ != solver_info_.limits.size()) {
		ROS_ERROR("Number of joints provided by the robot description and the IK solver information does not match");
	}
	
	q_min_.reset(new KDL::JntArray(number_of_joints_));
	q_max_.reset(new KDL::JntArray(number_of_joints_));

	for (unsigned int i = 0; i < number_of_joints_; i++) {
		if (solver_info_.limits[i].has_position_limits) {
			(*q_min_)(i) = solver_info_.limits[i].min_position;
			(*q_max_)(i) = solver_info_.limits[i].max_position;
		} else {
			ROS_ERROR("The IK solver information does not provide joint limits which are required by the constraint aware IK solver");
			ROS_ASSERT(false);
		}
	}
}


CobArmIKSolver::~CobArmIKSolver() {
}


void CobArmIKSolver::getSolverInfo(kinematics_msgs::KinematicSolverInfo &response)
{
	response = solver_info_;
}


/*
int CobArmIKSolver::CartToJnt(const KDL::JntArray& q_init, 
								const KDL::Frame& p_in, 
								KDL::JntArray &q_out)
{
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  if(free_angle_ == 0)
  {
    ROS_DEBUG("Solving with %f",q_init(0)); 
    pr2_arm_ik_.computeIKShoulderPan(b,q_init(0));
  }
  else
  {
    pr2_arm_ik_.computeIKShoulderRoll(b,q_init(2));
  }
  
  if(pr2_arm_ik_.solution_ik_.empty())
    return -1;

  double min_distance = 1e6;
  int min_index = -1;

  for(int i=0; i< (int) pr2_arm_ik_.solution_ik_.size(); i++)
  {     
    ROS_DEBUG("Solution : %d",(int)pr2_arm_ik_.solution_ik_.size());

    for(int j=0; j < (int)pr2_arm_ik_.solution_ik_[i].size(); j++)
    {   
      ROS_DEBUG("%d: %f",j,pr2_arm_ik_.solution_ik_[i][j]);
    }
    ROS_DEBUG(" ");
    ROS_DEBUG(" ");

    double tmp_distance = computeEuclideanDistance(pr2_arm_ik_.solution_ik_[i],q_init);
    if(tmp_distance < min_distance)
    {
      min_distance = tmp_distance;
      min_index = i;
    }
  }

  if(min_index > -1)
  {
    q_out.resize((int)pr2_arm_ik_.solution_ik_[min_index].size());
    for(int i=0; i < (int)pr2_arm_ik_.solution_ik_[min_index].size(); i++)
    {   
      q_out(i) = pr2_arm_ik_.solution_ik_[min_index][i];
    }
    return 1;
  }
  else
    return -1;

	return 0;
}
*/


int CobArmIKSolver::CartToJnt(const KDL::JntArray& q_init, 
                              const KDL::Frame& p_in, 
                              KDL::JntArray &q_out)
{
	// Forward position solver
	KDL::ChainFkSolverPos_recursive fksolver1(chain_);
	// Inverse velocity solver
	KDL::ChainIkSolverVel_pinv iksolver1v(chain_);
	// Maximum 100 iterations, stop at accuracy 1e-6
	KDL::ChainIkSolverPos_NR_JL iksolverpos(chain_, *q_min_, *q_max_, fksolver1, iksolver1v, 1000, 1e-6);

	// uhr-fm: here comes the actual IK-solver-call -> could be replaced by analytical-IK-solver (cob)
	int ret = iksolverpos.CartToJnt(q_init, p_in, q_out);

	ROS_DEBUG("q_init: %f %f %f %f %f %f %f", q_init(0), q_init(1), q_init(2), q_init(3), q_init(4), q_init(5), q_init(6));
	ROS_DEBUG("q_out: %f %f %f %f %f %f %f", q_out(0), q_out(1), q_out(2), q_out(3), q_out(4), q_out(5), q_out(6));	
	
	if (ret < 0) {
		ROS_DEBUG("Inverse Kinematic found no solution. KDL Return value = %i", ret);
		for (int i = 0; i < number_of_joints_; i++) q_out(i) = q_init(i);
		
		return -1;
	} else {
		ROS_INFO("Inverse Kinematic found a solution");
		
		return 1;
	}	
}


int CobArmIKSolver::CartToJnt(const KDL::JntArray& q_init, 
								const KDL::Frame& p_in, 
								std::vector<KDL::JntArray> &q_out)
{
/*
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  KDL::JntArray q;

  if(free_angle_ == 0)
  {
    pr2_arm_ik_.computeIKShoulderPan(b,q_init(0));
  }
  else
  {
    pr2_arm_ik_.computeIKShoulderRoll(b,q_init(2));
  }
  
  if(pr2_arm_ik_.solution_ik_.empty())
    return -1;

  q.resize(7);
  q_out.clear();
  for(int i=0; i< (int) pr2_arm_ik_.solution_ik_.size(); i++)
  {     
    for(int j=0; j < 7; j++)
    {   
      q(j) = pr2_arm_ik_.solution_ik_[i][j];
    }
    q_out.push_back(q);
  }
  return 1;
*/
	ROS_INFO("IK with redundant solutions not implemented for Care-O-bot");

	return 0;
}


bool CobArmIKSolver::getCount(int &count, 
								const int &max_count, 
								const int &min_count)
{
	if (count > 0) {
		if(-count >= min_count) {   
			count = -count;
			return true;
		} else if (count + 1 <= max_count) {
			count = count + 1;
			return true;
		} else {
			return false;
		}
	} else {
		if (1 - count <= max_count) {
			count = 1 - count;
			return true;
		} else if (count - 1 >= min_count) {
			count = count -1;
			return true;
		} else {
			return false;
		}
	}
}


int CobArmIKSolver::CartToJntSearch(const KDL::JntArray& q_in, 
										const KDL::Frame& p_in, 
										std::vector<KDL::JntArray> &q_out, 
										const double &timeout)
{
  KDL::JntArray q_init = q_in;
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((solver_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-solver_info_.limits[free_angle_].min_position)/search_discretization_angle_);
  ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,solver_info_.limits[free_angle_].max_position,solver_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);
  while(loop_time < timeout)
  {
    if(CartToJnt(q_init,p_in,q_out) > 0)
      return 1;
    if(!getCount(count,num_positive_increments,-num_negative_increments))
      return -1;
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_DEBUG("%d, %f",count,q_init(free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  if(loop_time >= timeout)
  {
    ROS_DEBUG("IK Timed out in %f seconds",timeout);
    return TIMED_OUT;
  }
  else
  {
    ROS_DEBUG("No IK solution was found");
    return NO_IK_SOLUTION;
  }
  return NO_IK_SOLUTION;
}


int CobArmIKSolver::CartToJntSearch(const KDL::JntArray& q_in, 
										const KDL::Frame& p_in, 
										KDL::JntArray &q_out, 
										const double &timeout)
{
  KDL::JntArray q_init = q_in;
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((solver_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-solver_info_.limits[free_angle_].min_position)/search_discretization_angle_);
  ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,solver_info_.limits[free_angle_].max_position,solver_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);
  while(loop_time < timeout)
  {
    if(CartToJnt(q_init,p_in,q_out) > 0)
      return 1;
    if(!getCount(count,num_positive_increments,-num_negative_increments))
      return -1;
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_DEBUG("%d, %f",count,q_init(free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  if(loop_time >= timeout)
  {
    ROS_DEBUG("IK Timed out in %f seconds",timeout);
    return TIMED_OUT;
  }
  else
  {
    ROS_DEBUG("No IK solution was found");
    return NO_IK_SOLUTION;
  }
  return NO_IK_SOLUTION;
}


int CobArmIKSolver::CartToJntSearch(const KDL::JntArray& q_in, 
										const KDL::Frame& p_in, 
										KDL::JntArray &q_out, 
										const double &timeout, 
										motion_planning_msgs::ArmNavigationErrorCodes &error_code,
										const boost::function<void(const KDL::JntArray&,const KDL::Frame&,motion_planning_msgs::ArmNavigationErrorCodes &)> &desired_pose_callback,
										const boost::function<void(const KDL::JntArray&,const KDL::Frame&,motion_planning_msgs::ArmNavigationErrorCodes &)> &solution_callback)
{
	ROS_INFO("CartToJntSearch in the IK solver has been called");

	Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
	KDL::JntArray q_init = q_in;
	double initial_guess = q_init(free_angle_);

	ros::Time start_time = ros::Time::now();
	double loop_time = 0;
	int count = 0;

	ROS_DEBUG("Calculating number of increments");
	ROS_DEBUG("Joint limits: %f - %f", solver_info_.limits[free_angle_].max_position, solver_info_.limits[free_angle_].min_position);
	
	int num_positive_increments = (int)((solver_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
	int num_negative_increments = (int)((initial_guess-solver_info_.limits[free_angle_].min_position)/search_discretization_angle_);
	ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,solver_info_.limits[free_angle_].max_position,solver_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);

	ROS_DEBUG("Checking initial pose");
	if (!desired_pose_callback.empty()) desired_pose_callback(q_init, p_in, error_code);
	if (error_code.val != error_code.SUCCESS) return -1;
	
	bool callback_check = true;
	if (solution_callback.empty()) callback_check = false;

	ROS_DEBUG("Searching for solutions");

	while (loop_time < timeout) {
		if (CartToJnt(q_init, p_in, q_out) > 0) {
			if (callback_check)  {
				solution_callback(q_out,p_in,error_code);
				if (error_code.val == error_code.SUCCESS) return 1;
		  	} else {
				error_code.val = error_code.SUCCESS;
				return 1;
			}
		}
		
		if(!getCount(count,num_positive_increments,-num_negative_increments)) {
			error_code.val = error_code.NO_IK_SOLUTION;
			return -1;
		}
		
		q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
		ROS_DEBUG("Redundancy search, index:%d, free angle value: %f", count, q_init(free_angle_));
		loop_time = (ros::Time::now() - start_time).toSec();
	}
	
	if(loop_time >= timeout) {
		ROS_DEBUG("IK Timed out in %f seconds",timeout);
		error_code.val = error_code.TIMED_OUT;
	} else {
		ROS_DEBUG("No IK solution was found");
		error_code.val = error_code.NO_IK_SOLUTION;
	}
	
	return -1;
}


std::string CobArmIKSolver::getFrameId()
{
	return root_frame_name_;
}

