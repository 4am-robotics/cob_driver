/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <ros/ros.h>
#include <iostream>
#include <cob_relayboard/GyroBoard.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

/*\brief SRBGyroNode is a wrapper class for converting the gyroboard output of the SerialRelayBoard into ROS sensor_msgs;
*/
class SRBGyroNode
{
	public:
	SRBGyroNode();
	ros::NodeHandle n;
	ros::Publisher topicPub_gyro;	
	ros::Subscriber topicSub_SRBgyro;
	
	int init();
	void sendGyroState(const cob_relayboard::GyroBoard& state);
	private:
	tf::TransformBroadcaster br;
	tf::Transform transform;
	std::string frame_name;
	std::vector<double> position, orientation;
	bool hasNoTFBroadcast;
};


int SRBGyroNode::init()
{
	n.getParam("frameName",frame_name);
	n.getParam("hasNoTFBroadcast",hasNoTFBroadcast);
	if(hasNoTFBroadcast) //no transformation broadcaster? --> use internal tf broadcaster
	{
		XmlRpc::XmlRpcValue e;
		n.getParam("translation",e);
		for(int i=0; i<e.size(); i++)
		{
			position.push_back((double) e[i]);
		}
		n.getParam("orientation",e);
		for(int i=0; i<e.size(); i++)
		{
			orientation.push_back((double) e[i]);
		}

		transform.setOrigin( tf::Vector3(position[0], position[1], position[2]) );
		transform.setRotation( tf::Quaternion(orientation[0], orientation[1], orientation[2]) );
	}
	topicSub_SRBgyro = n.subscribe("/srb_gyro_measurements",1,&SRBGyroNode::sendGyroState, this);
	topicPub_gyro = n.advertise<sensor_msgs::Imu>("/imu",1);
	ROS_INFO("started relayboard gyro wrapper");
	return 0;
}

void SRBGyroNode::sendGyroState(const cob_relayboard::GyroBoard& srsState)
{
	sensor_msgs::Imu state;
	state.header.stamp = ros::Time::now();
	state.header.frame_id = frame_name;
	state.orientation_covariance[0] = -1; state.orientation_covariance[4] = -1; state.orientation_covariance[8] = -1;
	state.angular_velocity_covariance[0] = -1; state.angular_velocity_covariance[4] = -1;
	state.angular_velocity.z = srsState.orientation;
	state.linear_acceleration.x = srsState.acceleration[0];
	state.linear_acceleration.y = srsState.acceleration[1];
	state.linear_acceleration.z = srsState.acceleration[2];
	topicPub_gyro.publish(state);
	if(hasNoTFBroadcast) br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", frame_name));
}


/**
*/
SRBGyroNode::SRBGyroNode () 
{


}



/**
*/
int main (int argc, char** argv)
{
	ros::init(argc, argv, "neo_SRBGyro_node");
	
	SRBGyroNode  node;
	if(node.init() != 0) return 1;
	ros::spin();
	return 0;
}
