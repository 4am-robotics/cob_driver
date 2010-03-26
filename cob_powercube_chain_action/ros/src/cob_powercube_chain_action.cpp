#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <cob_powercube_chain_action/JointTrajectoryAction.h>
#include <cob_powercube_chain_action/DoDishesAction.h>

class ControlHead
{
private:
  typedef actionlib::ActionServer<cob_powercube_chain_action::JointTrajectoryAction> PHAS;
  typedef PHAS::GoalHandle GoalHandle;
public:
  ControlHead(const ros::NodeHandle &n)
    : node_(n),
      action_server_(node_, "point_head_action",
                     boost::bind(&ControlHead::goalCB, this, _1),
                     boost::bind(&ControlHead::cancelCB, this, _1)),
      has_active_goal_(false)
  {
    ros::NodeHandle pn("~");
	topicPub_trajectory_ = node_.advertise<trajectory_msgs::JointTrajectory>("command", 2);
  }

  void goalCB(GoalHandle gh)
  {
    // Computes the command to send to the trajectory controller.
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();

    traj.joint_names.push_back("joint_platform_neckZ");
    traj.joint_names.push_back("joint_neckZ_neck");

	std::vector<double> pos,vel,pos2,vel2;
	pos.resize(2);
	vel.resize(2);
	pos2.resize(2);
	vel2.resize(2);

    traj.points.resize(2);
    traj.points[0].positions = pos;
    traj.points[0].velocities = vel;
    traj.points[0].time_from_start = ros::Duration(0.0);
    traj.points[1].positions = pos2;
    traj.points[1].velocities = vel2;
    traj.points[1].time_from_start = ros::Duration(1);

    topicPub_trajectory_.publish(traj);
  }

private:
  ros::NodeHandle node_;
  PHAS action_server_;
  bool has_active_goal_;
  GoalHandle active_goal_;
  ros::Publisher topicPub_trajectory_;
  
  void cancelCB(GoalHandle gh)
  {
    if (active_goal_ == gh)
    {
      // Stops the controller.
      trajectory_msgs::JointTrajectory empty;
      empty.joint_names.push_back("joint_platform_neckZ");
      empty.joint_names.push_back("joint_neckZ_neck");
      topicPub_trajectory_.publish(empty);

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_head_action");
  ros::NodeHandle node;
  ControlHead ch(node);
  ros::spin();
  return 0;
}
