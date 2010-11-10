#include "ros/ros.h"
#include "kinematics_msgs/GetPositionIK.h"
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

bool ik_solve(kinematics_msgs::GetPositionIK::Request  &req,
         kinematics_msgs::GetPositionIK::Response &res )
{
	ROS_INFO("ik_solve()-service call");
	
	unsigned int nj = chain.getNrOfJoints();
	
	JntArray q_min(nj);
	JntArray q_max(nj);
	for(int i = 0; i < nj; i+=2)
	{
		q_min(i) = -6.0;
		q_max(i) = 6.0;
	}
	for(int i = 1; i < nj; i+=2)
	{
		q_min(i) = -2.0;
		q_max(i) = 2.0;
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
	std::cout << "Calculated Position out of Configuration:\n";
	std::cout << F_ist <<"\n";

	//uhr-fm: here comes the actual IK-solver-call -> could be replaced by analytical-IK-solver (cob)
	int ret = iksolverpos.CartToJnt(q_init,F_dest,q);
	res.solution.joint_state.name = req.ik_request.ik_seed_state.joint_state.name;
	res.solution.joint_state.position.resize(nj);
	if(ret < 0)
	{
		res.error_code.val = -1;
		ROS_INFO("Inverse Kinematic found no solution");
		std::cout << "RET: " << ret << std::endl;
		for(int i = 0; i < nj; i++)	
			res.solution.joint_state.position[i] = q_init(i);
	}
	else
	{
		ROS_INFO("Inverse Kinematic found a solution");
		res.error_code.val = 1;
		for(int i = 0; i < nj; i++)	
			res.solution.joint_state.position[i] = q(i);
	}
	//std::cout << "q_init\n";
	ROS_DEBUG("q_init: %f %f %f %f %f %f %f", q_init(0), q_init(1), q_init(2), q_init(3), q_init(4), q_init(5), q_init(6));
	ROS_DEBUG("q_out: %f %f %f %f %f %f %f", q(0), q(1), q(2), q(3), q(4), q(5), q(6));		
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
	
	//transform GetConstraintAwarePositionIK-msgs to GetPositionIK-msgs
	request.ik_request=req.ik_request;
	request.timeout=req.timeout;
	//all other fields of GetConstraintAwarePositionIK::Request (allowed_contacts, ordered_collision_operations, link_padding, constraints) are dropped
	
	bool success = ik_solve(request, response);
	
	res.solution=response.solution;
	res.error_code=response.error_code;

	return true;
}




//uhr-fm: new service
bool my_get_ik_solver_info(kinematics_msgs::GetKinematicSolverInfo::Request  &req,
         kinematics_msgs::GetKinematicSolverInfo::Response &res )
{
	ROS_INFO("[TESTING]: my_get_ik_solver_service has been called!");
	
	unsigned int nj = chain.getNrOfJoints();
	//ROS_INFO("[TESTING]: nj=%d",nj);
	unsigned int nl = chain.getNrOfSegments();
	//ROS_INFO("[TESTING]: nl=%d",nl);

	//---setting up response

	//joint_names
	for(unsigned int i=0; i<nj; i++)
  {
		res.kinematic_solver_info.joint_names.push_back(chain.getSegment(i).getJoint().getName());
  }
	//limits
		//joint limits are only saved in KDL::ChainIkSolverPos_NR_JL iksolverpos -> ik_solve().....but this is not visible here!
	for(unsigned int i=0; i<nj; i++)
  {
//		res.kinematic_solver_info.limits[i].joint_name=chain.getSegment(i).getJoint().getName();
//		res.kinematic_solver_info.limits[i].has_position_limits=
//		res.kinematic_solver_info.limits[i].min_position=
//		res.kinematic_solver_info.limits[i].max_position=	
//		res.kinematic_solver_info.limits[i].has_velocity_limits=
//		res.kinematic_solver_info.limits[i].max_velocity=
//		res.kinematic_solver_info.limits[i].has_acceleration_limits=
//		res.kinematic_solver_info.limits[i].max_acceleration=
	}
	//link_names	
	for(unsigned int i=0; i<nl; i++)
  {
		res.kinematic_solver_info.link_names.push_back(chain.getSegment(i).getName());
  }


	return true;
}


/*
//uhr-messmerf: new service
//trying to embedd the analytical IK-solver from AnaKinematics (svn/cob)
bool ik_solve_analytical(kinematics_msgs::GetPositionIK::Request  &req,
         kinematics_msgs::GetPositionIK::Response &res )
{
	ROS_INFO("[TESTING]: ik_solve_analytical has been called!");
	unsigned int nj = chain.getNrOfJoints();
	
	JntArray q_min(nj);
	JntArray q_max(nj);
	for(int i = 0; i < nj; i+=2)
	{
		q_min(i) = -3.1;
		q_max(i) = 3.1;
	}
	for(int i = 1; i < nj; i+=2)
	{
		q_min(i) = -2.0;
		q_max(i) = 2.0;
	}

	
	ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
	ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
	ChainIkSolverPos_NR_JL iksolverpos(chain, q_min, q_max, fksolver1,iksolver1v,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

	JntArray q(nj);
	JntArray q_init(nj);
	for(int i = 0; i < nj; i++)
		q_init(i) = req.ik_request.ik_seed_state.joint_state.position[i];
	Frame F_dest;
	Frame F_ist;
	fksolver1.JntToCart(q_init, F_ist);
	tf::PoseMsgToKDL(req.ik_request.pose_stamped.pose, F_dest);
	std::cout << "Getting Goal\n";
	std::cout << F_dest;
	std::cout << "Calculated Position out of Configuration:\n";
	std::cout << F_ist <<"\n";

	//uhr-fm: here comes the actual IK-solver-call -> could be replaced by analytical-IK-solver (cob)
	int ret = iksolverpos.CartToJnt(q_init,F_dest,q);
	res.solution.joint_state.name = req.ik_request.ik_seed_state.joint_state.name;
	res.solution.joint_state.position.resize(nj);
	if(ret < 0)
	{
		res.error_code.val = -1;
		ROS_INFO("Inverse Kinematic found no solution");
		//std::cout << "RET: " << ret << std::endl;
		for(int i = 0; i < nj; i++)	
			res.solution.joint_state.position[i] = q_init(i);
	}
	else
	{
		ROS_INFO("Inverse Kinematic found a solution");
		res.error_code.val = 1;
		for(int i = 0; i < nj; i++)	
			res.solution.joint_state.position[i] = q(i);
	}
	//std::cout << "q_init\n";
	ROS_DEBUG("q_init: %f %f %f %f %f %f %f", q_init(0), q_init(1), q_init(2), q_init(3), q_init(4), q_init(5), q_init(6));
	ROS_DEBUG("q_out: %f %f %f %f %f %f %f", q(0), q(1), q(2), q(3), q(4), q(5), q(6));		
	//std::cout << "Solved with " << ret << " as return\n";
	//std::cout << q(0) << " " << q(1) << " " << q(2) << " " << q(3) << " " << q(4) << " " << q(5) << " " << q(6)  << "\n";	

	return true;
}
*/




int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_ik_solver");
	ros::NodeHandle n;
	KDL::Tree my_tree;
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
//	ros::ServiceServer get_ik_service = n.advertiseService("get_ik", ik_solve_analytical);
	ros::ServiceServer get_ik_solver_info_service = n.advertiseService("get_ik_solver_info", my_get_ik_solver_info);

	ROS_INFO("IK Server Running.");
	ros::spin();

	return 0;
}
