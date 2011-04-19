#include "ros/ros.h"
#include <kdl/jntarray.hpp>
#include <sensor_msgs/JointState.h>
#include <brics_actuator/JointVelocities.h>
#include <trajectory_msgs/JointTrajectory.h>


using namespace std;
using namespace KDL;

KDL::JntArray VirtualQ;
KDL::JntArray q;

ros::Time last;
bool started = false;

ros::Publisher arm_pub_;  //publish topic arm_controller/command

JntArray parseJointStates(std::vector<std::string> names, std::vector<double> positions)
{
	JntArray q_temp(7);
	int count = 0;
	for(unsigned int i = 0; i < names.size(); i++)
    {
			if(strncmp(names[i].c_str(), "arm_", 4) == 0)
			{
				q_temp(count) = positions[i];
				count++;
      }
    }
	if(!started)
	{
		VirtualQ = q_temp;
		started = true;
		last = ros::Time::now();

		ROS_INFO("Starting up controller with first configuration");
	}
	return q_temp;
}


void sendVel(JntArray q_dot)
{
	ros::Time now = ros::Time::now();
	double dt = now.toSec() - last.toSec();
	last = now;
	double horizon = 3.0*dt;
	//std::cout << dt << "\n";

	trajectory_msgs::JointTrajectory traj;
	traj.header.stamp = ros::Time::now()+ros::Duration(0.01);
	traj.joint_names.push_back("arm_1_joint");
	traj.joint_names.push_back("arm_2_joint");
	traj.joint_names.push_back("arm_3_joint");
	traj.joint_names.push_back("arm_4_joint");
	traj.joint_names.push_back("arm_5_joint");
	traj.joint_names.push_back("arm_6_joint");
	traj.joint_names.push_back("arm_7_joint");

	traj.points.resize(1);
	bool nonzero = false;
	for(int i = 0; i < 7; i++)
	{
		if(q_dot(i) != 0.0)
		{
			traj.points[0].positions.push_back(VirtualQ(i) + q_dot(i)*horizon);
			traj.points[0].velocities.push_back(q_dot(i));
			VirtualQ(i) += q_dot(i)*dt;
			nonzero = true;
		}
	}
	traj.points[0].time_from_start = ros::Duration(horizon);
	if(nonzero)
		arm_pub_.publish(traj);
}

void controllerStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	std::vector<std::string> names = msg->name;
	std::vector<double> positions = msg->position;
	q = parseJointStates(names,positions);
}

void velocityCallback(const brics_actuator::JointVelocities::ConstPtr& msg)
{
	KDL::JntArray q_dot;
	q_dot.resize(7);
	for(unsigned int j = 0; j < 7; j++)
	{
		q_dot(j) = msg->velocities.at(j).value;
	}
	sendVel(q_dot);

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_simulation_tester");
	ros::NodeHandle n;
	arm_pub_ = n.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command",1);
	ros::Subscriber sub = n.subscribe("/joint_states", 1, controllerStateCallback);
	ros::Subscriber sub_vc = n.subscribe("/arm_controller/command_vel", 1, velocityCallback);
	ros::spin();

	return 0;
}
