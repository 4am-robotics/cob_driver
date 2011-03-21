#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cob_pseudo_joint_state_publisher");

  ros::NodeHandle n;

  ros::Publisher publisher = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
  tf::TransformBroadcaster broadcaster;

  ros::Rate loop_rate(200);	//10Hz; rate in jointstate_aggregator is 100Hz

  while (ros::ok())
  {
    sensor_msgs::JointState msg;

	//compose msg
	msg.header.stamp = ros::Time::now();
	
	XmlRpc::XmlRpcValue JointName_param_;
	if (n.hasParam("PseudoJoints"))
	{
		n.getParam("PseudoJoints", JointName_param_);
	}
	else
	{
		ROS_ERROR("Parameter PseudoJoints not set");
	}

	for (int i = 0; i<JointName_param_.size(); i++ )
	{
		msg.name.push_back(JointName_param_[i]);
		if((std::string)JointName_param_[i]=="floating_rot_w")
			msg.position.push_back(1.0);
		else
			msg.position.push_back(0.0);
		msg.velocity.push_back(0.0);
		
	}


    publisher.publish(msg);

    broadcaster.sendTransform(	tf::StampedTransform(	tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
														ros::Time::now(),"/map", "/base_footprint"));

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

