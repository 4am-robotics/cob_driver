#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

typedef std::map<std::string, double> MapType;
MapType jointpositions;
MapType jointvelocities;


void jsCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	MapType::iterator iter = jointpositions.begin();
	for(int i = 0; i < msg->name.size(); i++)
	{
		iter = jointpositions.find(msg->name[i]);
    if (iter != jointpositions.end() )
			iter->second = msg->position[i];
		iter = jointvelocities.find(msg->name[i]);
    if (iter != jointvelocities.end() )
			iter->second = msg->velocity[i];
	}
  //ROS_INFO("updated");
}

sensor_msgs::JointState getJointStatesMessage()
{
	sensor_msgs::JointState msg;
	msg.header.stamp = ros::Time::now();
	msg.name.resize(jointpositions.size());
	msg.position.resize(jointpositions.size());
	msg.velocity.resize(jointvelocities.size());


	MapType::const_iterator end = jointpositions.end();
	MapType::iterator iter = jointvelocities.begin();
	int count = 0; 
  for (MapType::const_iterator it = jointpositions.begin(); it != end; ++it)
  {
		msg.name[count] = it->first;
		msg.position[count] = it->second;
		iter = jointvelocities.find(it->first);
    if (iter != jointvelocities.end() )
			msg.velocity[count] = iter->second;;
		count++;
	}
	return msg;
}

int main(int argc, char **argv)
{
	XmlRpc::XmlRpcValue JointName_param_;
	 

  ros::init(argc, argv, "cob_joint_state_aggregator");
  ros::NodeHandle n;

	if (n.hasParam("Joints"))
	{
		n.getParam("Joints", JointName_param_);
	}
	else
	{
		ROS_ERROR("Parameter Jointnames not set");
	}
	for (int i = 0; i<JointName_param_.size(); i++ )
	{
		jointpositions.insert(std::pair<std::string,double>((std::string)JointName_param_[i],0.0));
		jointvelocities.insert(std::pair<std::string,double>((std::string)JointName_param_[i],0.0));
	}

  ros::Rate loop_rate(10);

  ros::Subscriber sub = n.subscribe("/joint_states", 1, jsCallback);
	ros::Publisher topicPub_JointState_ = n.advertise<sensor_msgs::JointState>("/joint_states_combined", 1);

  while (ros::ok())
 	{  
		sensor_msgs::JointState msg = getJointStatesMessage();
		topicPub_JointState_.publish(msg);
 		ros::spinOnce();
	 	loop_rate.sleep();
	}

  return 0;
}
