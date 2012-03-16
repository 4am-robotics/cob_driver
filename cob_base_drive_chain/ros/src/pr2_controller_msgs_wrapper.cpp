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

/* wrapp pr2_controller_msgs to trajectory_msgs so that these msgs can be used by cob_base_drive_chain or cob_relayboard */

#include <ros/ros.h>
#include <iostream>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <pr2_controllers_msgs/JointTrajectoryActionGoal.h>
#include <pr2_controllers_msgs/JointTrajectoryGoal.h>


class Pr2CrtlMsgsWrapper
{
	public:
	Pr2CrtlMsgsWrapper();
	ros::NodeHandle n;
	ros::Publisher topicPub_to;	
	ros::Subscriber topicSub_from;
	
	int init();
	void wrapFromJTControllerState(const pr2_controllers_msgs::JointTrajectoryControllerState& wrap_me);
	void wrapFromJTGoal(const pr2_controllers_msgs::JointTrajectoryGoal& wrap_me);
	void wrapFromJTActionGoal(const pr2_controllers_msgs::JointTrajectoryActionGoal& wrap_me);

	void wrapToJTControllerState(const trajectory_msgs::JointTrajectory& wrap_me);
	void wrapToJTGoal(const trajectory_msgs::JointTrajectory& wrap_me);
	void wrapToJTActionGoal(const trajectory_msgs::JointTrajectory& wrap_me);
};


int Pr2CrtlMsgsWrapper::init()
{
	int wrap_type;
	n.param<int>("wrap_type", wrap_type, 0);
	switch (wrap_type)
	{
		case 0:
			topicSub_from = n.subscribe("/from",1,&Pr2CrtlMsgsWrapper::wrapFromJTControllerState, this);
			topicPub_to = n.advertise<trajectory_msgs::JointTrajectory>("/to",1);
			break;
		case 1:
			topicSub_from = n.subscribe("/from",1,&Pr2CrtlMsgsWrapper::wrapFromJTGoal, this);
			topicPub_to = n.advertise<trajectory_msgs::JointTrajectory>("/to",1);
			break;

		case 2:
			topicSub_from = n.subscribe("/from",1,&Pr2CrtlMsgsWrapper::wrapFromJTActionGoal, this);
			topicPub_to = n.advertise<trajectory_msgs::JointTrajectory>("/to",1);
			break;

		case 3:
			topicSub_from = n.subscribe("/from",1,&Pr2CrtlMsgsWrapper::wrapToJTControllerState, this);
			topicPub_to = n.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("/to",1);
			break;

		case 4:
			topicSub_from = n.subscribe("/from",1,&Pr2CrtlMsgsWrapper::wrapToJTGoal, this);
			topicPub_to = n.advertise<pr2_controllers_msgs::JointTrajectoryGoal>("/to",1);
			break;
		case 5:
			topicSub_from = n.subscribe("/from",1,&Pr2CrtlMsgsWrapper::wrapToJTActionGoal, this);
			topicPub_to = n.advertise<pr2_controllers_msgs::JointTrajectoryActionGoal>("/to",1);
			break;




	};


	ROS_INFO("started controller msgs wrapper");
	return 0;
}

void Pr2CrtlMsgsWrapper::wrapFromJTControllerState(const pr2_controllers_msgs::JointTrajectoryControllerState& wrap_me)
{
	trajectory_msgs::JointTrajectory wrap_to;
	wrap_to.header = wrap_me.header;
	wrap_to.joint_names = wrap_me.joint_names;
	wrap_to.points.resize(1);
	wrap_to.points[0] = wrap_me.desired;
	//information lost: actual and error.
	topicPub_to.publish(wrap_to);
};

void Pr2CrtlMsgsWrapper::wrapFromJTGoal(const pr2_controllers_msgs::JointTrajectoryGoal& wrap_me)
{
	trajectory_msgs::JointTrajectory wrap_to;
	wrap_to = wrap_me.trajectory;
	//information lost: non.
	topicPub_to.publish(wrap_to);
};

void Pr2CrtlMsgsWrapper::wrapFromJTActionGoal(const pr2_controllers_msgs::JointTrajectoryActionGoal& wrap_me)
{
	trajectory_msgs::JointTrajectory wrap_to;
	wrap_to.header = wrap_me.header;
	wrap_to = wrap_me.goal.trajectory;
	//information lost: goal_id .
	topicPub_to.publish(wrap_to);

};

void Pr2CrtlMsgsWrapper::wrapToJTControllerState(const trajectory_msgs::JointTrajectory& wrap_me)
{
	pr2_controllers_msgs::JointTrajectoryControllerState wrap_to;
	wrap_to.header = wrap_me.header;
	wrap_to.desired = wrap_me.points[0];
	wrap_to.joint_names = wrap_me.joint_names;
	//information not available: actual and error
	topicPub_to.publish(wrap_to);
};

void Pr2CrtlMsgsWrapper::wrapToJTGoal(const trajectory_msgs::JointTrajectory& wrap_me)
{
	pr2_controllers_msgs::JointTrajectoryGoal wrap_to;
	wrap_to.trajectory = wrap_me;
	//information not available: non
	topicPub_to.publish(wrap_to);
};

void Pr2CrtlMsgsWrapper::wrapToJTActionGoal(const trajectory_msgs::JointTrajectory& wrap_me)
{
	pr2_controllers_msgs::JointTrajectoryActionGoal wrap_to;
	wrap_to.header = wrap_me.header;
	wrap_to.goal.trajectory = wrap_me;
	//information lost: goal_id .
	topicPub_to.publish(wrap_to);
};

/**
*/
Pr2CrtlMsgsWrapper::Pr2CrtlMsgsWrapper () 
{


}



/**
*/
int main (int argc, char** argv)
{
	ros::init(argc, argv, "neo_controller_msgs_wrapper_node");
	
	Pr2CrtlMsgsWrapper  node;
	if(node.init() != 0) return 1;
	ros::spin();
	return 0;
}
