#include "ros/ros.h"
#include "kinematics_msgs/GetPositionIK.h"
#include <kdl_parser/kdl_parser.hpp>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_kdl.h>

#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>

using namespace std;
using namespace KDL;
KDL::Chain chain;

bool ik_solve(kinematics_msgs::GetPositionIK::Request  &req,
         kinematics_msgs::GetPositionIK::Response &res )
{
	ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
	ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
	ChainIkSolverPos_NR iksolverpos(chain,fksolver1,iksolver1v,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

	std::cout << "Created Solver\n";
	unsigned int nj = chain.getNrOfJoints();
	std::cout << "Chain has " << nj << " Joints\n";

	JntArray q(nj);
	JntArray q_init(nj);
	for(int i = 0; i < nj; i++)
		q_init(i) = req.ik_request.ik_seed_state.joint_state.position[i];
	Frame F_dest;
	Frame F_ist;
	fksolver1.JntToCart(q_init, F_ist);
	tf::PoseMsgToKDL(req.ik_request.pose_stamped.pose, F_dest);
	//std::cout << "Getting Goal\n";
	//std::cout << F_dest <<"\n";
	//std::cout << "Calculated Position out of Configuration:\n";
	//std::cout << F_ist <<"\n";

	int ret = iksolverpos.CartToJnt(q_init,F_dest,q);
	//std::cout << "q_init\n";
	//std::cout << q_init(0) << " " << q_init(1) << " " << q_init(2) << " " << q_init(3) << " " << q_init(4) << " " << q_init(5) << " " << q_init(6) << "\n";	
	//std::cout << "Solved with " << ret << " as return\n";
	//std::cout << q(0) << " " << q(1) << " " << q(2) << " " << q(3) << " " << q(4) << " " << q(5) << " " << q(6)  << "\n";	
	res.solution.joint_state.name = req.ik_request.ik_seed_state.joint_state.name;
	res.solution.joint_state.position.resize(nj);
	for(int i = 0; i < nj; i++)	
		res.solution.joint_state.position[i] = q(i);

	ROS_INFO("IK_Solver Called");
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
