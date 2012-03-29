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
#include <cob_relayboard/DriveStates.h>
#include <cob_relayboard/DriveCommands.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <boost/thread.hpp>

/*\brief SRBDriveNode is a wrapper class for converting the drive output of the SerialRelayBoard into ROS sensor_msgs;
	It doesn't support jointrajectories with multiple points.
*/
class SRBDriveNode
{
	public:
	SRBDriveNode();
	ros::NodeHandle n;
	ros::Publisher topicPub_drives;	
	ros::Subscriber topicSub_drives;
	ros::Publisher topicPub_SRBdrives;	
	ros::Subscriber topicSub_SRBdrives;
	
	int init();
	void sendJointState(const cob_relayboard::DriveStates& state);
	void sendDriveCommands(const trajectory_msgs::JointTrajectory& newState);
	private:
	boost::mutex mOut;


};


int SRBDriveNode::init()
{
	topicSub_SRBdrives = n.subscribe("/drive_states",1,&SRBDriveNode::sendJointState, this);
	topicPub_drives = n.advertise<sensor_msgs::JointState>("/joint_states",1);

	topicPub_SRBdrives = n.advertise<cob_relayboard::DriveCommands>("/cmd_drives",1);
        topicSub_drives = n.subscribe("/cmd_joint_traj",1,&SRBDriveNode::sendDriveCommands, this);
	ROS_INFO("started relayboard drive wrapper");
	return 0;
}

void SRBDriveNode::sendJointState(const cob_relayboard::DriveStates& srsState)
{
	//ROS_INFO("wrap srs velocity cmd: %f %f",srsState.angularVelocity[0],srsState.angularVelocity[1]);
	sensor_msgs::JointState state;
	state.header.stamp = ros::Time::now();
	for(int i=0; i<2; i++)
	{
		state.position.push_back(srsState.angularPosition[i]); 
		state.velocity.push_back(srsState.angularVelocity[i]);
		state.name.push_back(srsState.joint_names[i]);
	}  
	topicPub_drives.publish(state);
	//TODO: state.effort

}

void SRBDriveNode::sendDriveCommands(const trajectory_msgs::JointTrajectory& newState)
{
	//ROS_INFO("wrap velocity cmd: %f %f",newState.velocities[0],newState.velocities[1]);
	cob_relayboard::DriveCommands cmd;
	for(int i=0; i<2; i++)
	{
		cmd.angularVelocity[i] = newState.points[0].velocities[i];
		cmd.driveActive[i] = true;
		cmd.quickStop[i] = false;
		cmd.disableBrake[i] = true;
	}
	//TODO: brake:
	topicPub_SRBdrives.publish(cmd);
}

/**
*/
SRBDriveNode::SRBDriveNode () 
{


}



/**
*/
int main (int argc, char** argv)
{
	ros::init(argc, argv, "neo_SRBDrive_node");
	
	SRBDriveNode  node;
	if(node.init() != 0) return 1;
	ros::spin();
	return 0;
}
