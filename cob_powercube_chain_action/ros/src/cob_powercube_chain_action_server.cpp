#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cob_powercube_chain_action/JointTrajectoryAction.h>

#include <trajectory_msgs/JointTrajectory.h>

class JointTrajectoryAction
{
protected:

  ros::NodeHandle nh_;
  ros::Publisher topicPub_JointCommand;
  actionlib::SimpleActionServer<cob_powercube_chain_action::JointTrajectoryAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  cob_powercube_chain_action::JointTrajectoryFeedback feedback_;
  cob_powercube_chain_action::JointTrajectoryResult result_;

public:

  JointTrajectoryAction(std::string name) :
    as_(nh_, name, boost::bind(&JointTrajectoryAction::executeCB, this, _1)),
    action_name_(name)
  {
  	topicPub_JointCommand = nh_.advertise<trajectory_msgs::JointTrajectory>("command", 1);
  }

  ~JointTrajectoryAction(void)
  {
  }

  void executeCB(const cob_powercube_chain_action::JointTrajectoryGoalConstPtr &goal)
  {
    // helper variables
    bool success = true;

    // push_back the seeds for the JointTrajectory sequence
    feedback_.fb = -1;

    // publish info to the console for the user
    ROS_INFO("%s: Executing JointTrajectory", action_name_.c_str());

    // start executing the action
    for(int i=1; i<=goal->trajectory.points.size(); i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      
      // do some stuff
      topicPub_JointCommand.publish(goal->trajectory);
      
      feedback_.fb = i;
      // publish the feedback
      as_.publishFeedback(feedback_);
    }

    if(success)
    {
      result_.res = feedback_.fb;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "JointTrajectory");

//  JointTrajectoryAction JointTrajectory(ros::this_node::getName());
  JointTrajectoryAction JointTrajectory("JointTrajectory");
  ros::spin();

  return 0;
}

