#include "ros/ros.h"
#include "kinematics_msgs/GetPositionIK.h"
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
	

	ros::ServiceServer service = n.advertiseService("get_ik", ik_solve);
	ROS_INFO("IK Server Running.");
	ros::spin();

	return 0;
}
