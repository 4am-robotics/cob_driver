#include "ros/ros.h"
#include "kinematics_msgs/GetPositionIK.h"
#include "kinematics_msgs/GetPositionFK.h"
#include "kinematics_msgs/GetConstraintAwarePositionIK.h"
#include "kinematics_msgs/GetKinematicSolverInfo.h"
#include <kdl_parser/kdl_parser.hpp>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_kdl.h>

#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>

using namespace std;
using namespace KDL;
KDL::Chain chain;
KDL::Tree my_tree;

void getKDLChainInfo(kinematics_msgs::KinematicSolverInfo &chain_info)
{
	unsigned int nj = chain.getNrOfJoints();
	unsigned int nl = chain.getNrOfSegments();
	
	ROS_DEBUG("nj: %d", nj);
	ROS_DEBUG("nl: %d", nl);

	//---setting up response

	//joint_names
	for(unsigned int i=0; i<nj; i++)
	{
		ROS_DEBUG("joint_name[%d]: %s", i, chain.getSegment(i).getJoint().getName().c_str());
		chain_info.joint_names.push_back(chain.getSegment(i).getJoint().getName());
	}
	//limits
		//joint limits are only saved in KDL::ChainIkSolverPos_NR_JL iksolverpos -> ik_solve().....but this is not visible here!
/*for(unsigned int i=0; i<nj; i++)
  {
		chain_info.limits[i].joint_name=chain.getSegment(i).getJoint().getName();
		chain_info.limits[i].has_position_limits=
		chain_info.limits[i].min_position=
		chain_info.limits[i].max_position=	
		chain_info.limits[i].has_velocity_limits=
		chain_info.limits[i].max_velocity=
		chain_info.limits[i].has_acceleration_limits=
		chain_info.limits[i].max_acceleration=
	}*/
	//link_names	
	for(unsigned int i=0; i<nl; i++)
	{
		chain_info.link_names.push_back(chain.getSegment(i).getName());
	}
}

int getJointIndex(const std::string &name,
                  const kinematics_msgs::KinematicSolverInfo &chain_info)
  {
    for(unsigned int i=0; i < chain_info.joint_names.size(); i++)
    {
      if(chain_info.joint_names[i] == name)
      {
          return i;
      }
    }
    return -1;
  }


bool ik_solve(kinematics_msgs::GetPositionIK::Request  &req,
         kinematics_msgs::GetPositionIK::Response &res )
{
	ROS_INFO("get_ik_service has been called!");
	
	if(req.ik_request.ik_link_name.length() == 0)
		my_tree.getChain("base_link","arm_7_link", chain);
	else
		my_tree.getChain("base_link",req.ik_request.ik_link_name, chain);
	
	
	
	unsigned int nj = chain.getNrOfJoints();
	
	JntArray q_min(nj);
	JntArray q_max(nj);
	
	//ToDo: get joint limits from robot_description on parameter_server or use /cob_arm_kinematics/get_ik_solver_info!!!
	for(int i = 0; i < nj; i+=2)
	{
		q_min(i) =-2.9670;		//adjusted due to cob_description/lbr.urdf.xacro
		q_max(i) = 2.9670;
	}
	for(int i = 1; i < nj; i+=2)
	{
		q_min(i) =-2.0943951;	//adjusted due to cob_description/lbr.urdf.xacro
		q_max(i) = 2.0943951;
	}
	
	ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
	ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
	ChainIkSolverPos_NR_JL iksolverpos(chain, q_min, q_max, fksolver1,iksolver1v,1000,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

	JntArray q(nj);
	JntArray q_init(nj);
	for(int i = 0; i < nj; i++)
		q_init(i) = req.ik_request.ik_seed_state.joint_state.position[i];
	Frame F_dest;
	Frame F_ist;
	fksolver1.JntToCart(q_init, F_ist);
	tf::PoseMsgToKDL(req.ik_request.pose_stamped.pose, F_dest);
	std::cout << "Getting Goal\n";
	std::cout << F_dest <<"\n";
	std::cout << "Calculated Position out of Seed-Configuration:\n";
	std::cout << F_ist <<"\n";

	//uhr-fm: here comes the actual IK-solver-call -> could be replaced by analytical-IK-solver (cob)
	int ret = iksolverpos.CartToJnt(q_init,F_dest,q);
	
	//res.solution.joint_state.name = req.ik_request.ik_seed_state.joint_state.name;	
	res.solution.joint_state.name.resize(nj);
	res.solution.joint_state.name[0]="arm_1_joint";
	res.solution.joint_state.name[1]="arm_2_joint";
	res.solution.joint_state.name[2]="arm_3_joint";
	res.solution.joint_state.name[3]="arm_4_joint";
	res.solution.joint_state.name[4]="arm_5_joint";
	res.solution.joint_state.name[5]="arm_6_joint";
	res.solution.joint_state.name[6]="arm_7_joint";
	
	res.solution.joint_state.position.resize(nj);
	res.solution.joint_state.velocity.resize(nj);
	res.solution.joint_state.effort.resize(nj);
	if(ret < 0)
	{
		//res.error_code.val = 0;
		res.error_code.val = res.error_code.NO_IK_SOLUTION;
		ROS_INFO("Inverse Kinematic found no solution");
		std::cout << "RET: " << ret << std::endl;
		for(int i = 0; i < nj; i++)
		{
			res.solution.joint_state.position[i] = q_init(i);
			res.solution.joint_state.velocity[i] = 0.0;
			res.solution.joint_state.effort[i] = 0.0;
		}
	}
	else
	{
		ROS_INFO("Inverse Kinematic found a solution");
		//res.error_code.val = 1;
		res.error_code.val = res.error_code.SUCCESS;
		for(int i = 0; i < nj; i++)
		{
			res.solution.joint_state.position[i] = q(i);
			res.solution.joint_state.velocity[i] = 0.0;
			res.solution.joint_state.effort[i] = 0.0;
		}
	}
	//std::cout << "q_init\n";
	ROS_INFO("q_init: %f %f %f %f %f %f %f", q_init(0), q_init(1), q_init(2), q_init(3), q_init(4), q_init(5), q_init(6));
	ROS_INFO("q_out: %f %f %f %f %f %f %f", q(0), q(1), q(2), q(3), q(4), q(5), q(6));		
	//std::cout << "Solved with " << ret << " as return\n";
	//std::cout << q(0) << " " << q(1) << " " << q(2) << " " << q(3) << " " << q(4) << " " << q(5) << " " << q(6)  << "\n";	

	return true;
}


//uhr-fm: new service
bool constraint_aware_ik_solve(kinematics_msgs::GetConstraintAwarePositionIK::Request  &req,
         kinematics_msgs::GetConstraintAwarePositionIK::Response &res )
{
	kinematics_msgs::GetPositionIK::Request request;
	kinematics_msgs::GetPositionIK::Response response;
	
	
	//ToDo: configure pr2_arm_kinematics_constraint_aware for care-o-bot instead of this service (it is NOT constraint aware!!!)
	
	//transform GetConstraintAwarePositionIK-msgs to GetPositionIK-msgs
	request.ik_request=req.ik_request;
	request.timeout=req.timeout;
	//all other fields of GetConstraintAwarePositionIK::Request (allowed_contacts, ordered_collision_operations, link_padding, constraints) are dropped
	
	bool success = ik_solve(request, response);
	
	if(response.error_code.val == 1) res.error_code.val = res.error_code.SUCCESS;
	else res.error_code.val = res.error_code.NO_IK_SOLUTION;
	
	res.solution=response.solution;
	res.solution.multi_dof_joint_state = arm_navigation_msgs::MultiDOFJointState();

	return true;
}


bool getIKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request  &req,
         kinematics_msgs::GetKinematicSolverInfo::Response &res )
{
	ROS_INFO("[TESTING]: get_ik_solver_info_service has been called!");
	
	///ToDo: this call returns joint_names "arm_i_joint", where i=0...6. should be: i=1...7!!!!
	getKDLChainInfo(res.kinematic_solver_info);

	return true;
}

bool fk_solve_TCP(kinematics_msgs::GetPositionFK::Request &req,
		 kinematics_msgs::GetPositionFK::Response &res )
{
	ROS_INFO("[TESTING]: get_fk_TCP_service has been called!");

	unsigned int nj = chain.getNrOfJoints();
	
	ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver

	JntArray conf(nj);
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose;

	for(int i = 0; i < nj; i++)
		conf(i) = req.robot_state.joint_state.position[i];
	Frame F_ist;
	res.pose_stamped.resize(1);
	res.fk_link_names.resize(1);
	if(fksolver1.JntToCart(conf, F_ist) >= 0)
	{
		std::cout << "Calculated Position out of Configuration:\n";
		std::cout << F_ist <<"\n";
	
		//TODO: fill out response!!!

        tf_pose.frame_id_ = "base_link";//root_name_;
        tf_pose.stamp_ = ros::Time();
        tf::PoseKDLToTF(F_ist,tf_pose);
        try
		{
          //tf_.transformPose(req.header.frame_id,tf_pose,tf_pose);
        }
        catch(...)
        {
          ROS_ERROR("Could not transform FK pose to frame: %s",req.header.frame_id.c_str());
          res.error_code.val = res.error_code.FRAME_TRANSFORM_FAILURE;
          return false;
        }
        tf::poseStampedTFToMsg(tf_pose,pose);
        res.pose_stamped[0] = pose;
        res.fk_link_names[0] = "arm_7_link";
        res.error_code.val = res.error_code.SUCCESS;
	}
	else
	{
        ROS_ERROR("Could not compute FK for arm_7_link");
        res.error_code.val = res.error_code.NO_FK_SOLUTION;
	}

	return true;
}

bool fk_solve_all(kinematics_msgs::GetPositionFK::Request &req,
		  kinematics_msgs::GetPositionFK::Response &res )
{
	ROS_INFO("[TESTING]: get_fk_all_service has been called!");

	unsigned int nj = chain.getNrOfJoints();
	unsigned int nl = chain.getNrOfSegments();
	
	ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver

	JntArray conf(nj);
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose;

	for(int i = 0; i < nj; i++)
		conf(i) = req.robot_state.joint_state.position[i];
	Frame F_ist;
		res.pose_stamped.resize(nl);
	res.fk_link_names.resize(nl);
	for(int j = 0; j < nl; j++)
	{
		if(fksolver1.JntToCart(conf, F_ist, j+1) >= 0)
		{
		std::cout << "Calculated Position out of Configuration for segment " << j+1 << ":\n";
			std::cout << F_ist <<"\n";
		
			std::cout << "Calculated Position out of Configuration:\n";
			std::cout << F_ist <<"\n";
	
			//TODO: fill out response!!!

		    tf_pose.frame_id_ = "base_link";//root_name_;
		    tf_pose.stamp_ = ros::Time();
		    tf::PoseKDLToTF(F_ist,tf_pose);
		    try
			{
		      //tf_.transformPose(req.header.frame_id,tf_pose,tf_pose);
		    }
		    catch(...)
		    {
		      ROS_ERROR("Could not transform FK pose to frame: %s",req.header.frame_id.c_str());
		      res.error_code.val = res.error_code.FRAME_TRANSFORM_FAILURE;
		      return false;
		    }
		    tf::poseStampedTFToMsg(tf_pose,pose);
		    res.pose_stamped[j] = pose;
		    res.fk_link_names[j] = chain.getSegment(j).getName();
		    res.error_code.val = res.error_code.SUCCESS;
		}
		else
		{
		    ROS_ERROR("Could not compute FK for segment %d", j);
		    res.error_code.val = res.error_code.NO_FK_SOLUTION;
		}


		//TODO: fill out response!!!
		//res.pose_stamped[j]
	}

	return true;
}


bool fk_solve(kinematics_msgs::GetPositionFK::Request  &req,
         kinematics_msgs::GetPositionFK::Response &res )
{
	ROS_INFO("[TESTING]: get_fk_service has been called!");

//from pr2_arm_kinematics
/*	if(!active_)
    {
      ROS_ERROR("FK service not active");
      return true;
    }

    if(!checkFKService(request,response,fk_solver_info_))
      return true;

    KDL::Frame p_out;
    KDL::JntArray jnt_pos_in;
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose;

    jnt_pos_in.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      int tmp_index = getJointIndex(request.robot_state.joint_state.name[i],fk_solver_info_);
      if(tmp_index >=0)
        jnt_pos_in(tmp_index) = request.robot_state.joint_state.position[i];
    }

    response.pose_stamped.resize(request.fk_link_names.size());
    response.fk_link_names.resize(request.fk_link_names.size());

    bool valid = true;
    for(unsigned int i=0; i < request.fk_link_names.size(); i++)
    {
      ROS_DEBUG("End effector index: %d",pr2_arm_kinematics::getKDLSegmentIndex(kdl_chain_,request.fk_link_names[i]));
      ROS_DEBUG("Chain indices: %d",kdl_chain_.getNrOfSegments());
      if(jnt_to_pose_solver_->JntToCart(jnt_pos_in,p_out,pr2_arm_kinematics::getKDLSegmentIndex(kdl_chain_,request.fk_link_names[i])) >=0)
      {
        tf_pose.frame_id_ = root_name_;
        tf_pose.stamp_ = ros::Time();
        tf::PoseKDLToTF(p_out,tf_pose);
        try{
          tf_.transformPose(request.header.frame_id,tf_pose,tf_pose);
        }
        catch(...)
        {
          ROS_ERROR("Could not transform FK pose to frame: %s",request.header.frame_id.c_str());
          response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
          return false;
        }
        tf::poseStampedTFToMsg(tf_pose,pose);
        response.pose_stamped[i] = pose;
        response.fk_link_names[i] = request.fk_link_names[i];
        response.error_code.val = response.error_code.SUCCESS;
      }
      else
      {
        ROS_ERROR("Could not compute FK for %s",request.fk_link_names[i].c_str());
        response.error_code.val = response.error_code.NO_FK_SOLUTION;
        valid = false;
      }
    }
*/
    return true;
}	


bool getFKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &req, 
                                     kinematics_msgs::GetKinematicSolverInfo::Response &res)
{
	ROS_INFO("[TESTING]: get_fk_solver_info_service has been called!");

	getKDLChainInfo(res.kinematic_solver_info);

	return true;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_ik_solver");
	ros::NodeHandle n;
	ros::NodeHandle node;
	std::string robot_desc_string;
	node.param("/robot_description", robot_desc_string, string());
	if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
	      ROS_ERROR("Failed to construct kdl tree");
      	      return false;
	}
	my_tree.getChain("base_link","arm_7_link", chain);


	ros::ServiceServer get_ik_service = n.advertiseService("get_ik", ik_solve);
	ros::ServiceServer get_constraint_aware_ik_service = n.advertiseService("get_constraint_aware_ik", constraint_aware_ik_solve);
	ros::ServiceServer get_ik_solver_info_service = n.advertiseService("get_ik_solver_info", getIKSolverInfo);
	ros::ServiceServer get_fk_service = n.advertiseService("get_fk", fk_solve);
	ros::ServiceServer get_fk_tcp_service = n.advertiseService("get_fk_tcp", fk_solve_TCP);
	ros::ServiceServer get_fk_all_service = n.advertiseService("get_fk_all", fk_solve_all);
	ros::ServiceServer get_fk_solver_info_service = n.advertiseService("get_fk_solver_info", getFKSolverInfo);


	ROS_INFO("IK Server Running.");
	ros::spin();

	return 0;
}
