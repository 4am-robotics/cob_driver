#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include "std_msgs/Float64.h"

ros::Publisher br_steer_pub;
ros::Publisher bl_steer_pub;
ros::Publisher fr_steer_pub;
ros::Publisher fl_steer_pub;

ros::Publisher br_caster_pub;
ros::Publisher bl_caster_pub;
ros::Publisher fr_caster_pub;
ros::Publisher fl_caster_pub;


void velCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
	if(msg->desired.velocities.size() != 8)
		return;
	std_msgs::Float64 fl;
	for(unsigned int i = 0; i < 8; i++)
	{
		fl.data = msg->desired.velocities[i];
		if(msg->joint_names[i] == "fl_caster_r_wheel_joint")
			fl_caster_pub.publish(fl);
		if(msg->joint_names[i] == "fr_caster_r_wheel_joint")
			fr_caster_pub.publish(fl);
		if(msg->joint_names[i] == "bl_caster_r_wheel_joint")
			bl_caster_pub.publish(fl);
		if(msg->joint_names[i] == "br_caster_r_wheel_joint")
			br_caster_pub.publish(fl);

		if(msg->joint_names[i] == "fl_caster_rotation_joint")
			fl_steer_pub.publish(fl);
		if(msg->joint_names[i] == "fr_caster_rotation_joint")
			fr_steer_pub.publish(fl);
		if(msg->joint_names[i] == "bl_caster_rotation_joint")
			bl_steer_pub.publish(fl);
		if(msg->joint_names[i] == "br_caster_rotation_joint")
			br_steer_pub.publish(fl);
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_drive_chain_sim");
  ros::NodeHandle n;
  bl_caster_pub = n.advertise<std_msgs::Float64>("/base_bl_caster_r_wheel_controller/command", 1);
  br_caster_pub = n.advertise<std_msgs::Float64>("/base_br_caster_r_wheel_controller/command", 1);
  fl_caster_pub = n.advertise<std_msgs::Float64>("/base_fl_caster_r_wheel_controller/command", 1);
  fr_caster_pub = n.advertise<std_msgs::Float64>("/base_fr_caster_r_wheel_controller/command", 1);

  bl_steer_pub = n.advertise<std_msgs::Float64>("/base_bl_caster_rotation_controller/command", 1);
  br_steer_pub = n.advertise<std_msgs::Float64>("/base_br_caster_rotation_controller/command", 1);
  fl_steer_pub = n.advertise<std_msgs::Float64>("/base_fl_caster_rotation_controller/command", 1);
  fr_steer_pub = n.advertise<std_msgs::Float64>("/base_fr_caster_rotation_controller/command", 1);


  ros::Subscriber sub = n.subscribe("joint_command", 1, velCallback);
  ros::spin();

  return 0;
}
