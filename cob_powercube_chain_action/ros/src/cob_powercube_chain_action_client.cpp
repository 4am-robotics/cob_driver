#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cob_powercube_chain_action/JointTrajectoryAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_powercube_client"); 

  // create the action client
  // true causes the client to spin it's own thread
  actionlib::SimpleActionClient<cob_powercube_chain_action::JointTrajectoryAction> ac("JointTrajectory", true); 

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
 
  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action 
  cob_powercube_chain_action::JointTrajectoryGoal goal;
  
  trajectory_msgs::JointTrajectory traj;
  traj.header.stamp = ros::Time::now();
  traj.points.resize(2);
  traj.points[0].positions.resize(4);
  traj.points[0].velocities.resize(4);
  traj.points[1].positions.resize(4);
  traj.points[1].velocities.resize(4);
  
  traj.points[0].positions[2] = 0.2;
  
  traj.points[1].positions[2] = -0.2;
  
  goal.trajectory = traj;
  ac.sendGoal(goal);
  
  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else  
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}

