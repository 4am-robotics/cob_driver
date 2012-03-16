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
#include <cob_relayboard/IRSensors.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <XmlRpcValue.h>


/*\brief SRB_IR_Node is a wrapper class for converting the _irboard output of the SerialRelayBoard into ROS sensor_msgs;
*/
class SRB_IR_Node
{
	public:
	SRB_IR_Node();
	ros::NodeHandle n;
	ros::Publisher topicPub_ir;	
	ros::Subscriber topicSub_SRB_ir;
	
	int init();
	void send_IR_State(const cob_relayboard::IRSensors& state);
	std::vector<bool> activeSensors;
	std::vector<int> sensor_type;
	std::vector<std::string> frame_name;
	std::vector<double> minRange, maxRange, field_of_view;
	std::vector< std::vector<double> > position, orientation;

};


int SRB_IR_Node::init()
{
	topicSub_SRB_ir = n.subscribe("/srb_ir_measurements",1,&SRB_IR_Node::send_IR_State, this);

	XmlRpc::XmlRpcValue a;
	n.getParam("radiationType",a);
	for(int i=0; i<a.size(); i++)
	{	
		sensor_type.push_back((int) a[i]);
	}

	n.getParam("frameName",a);
	for(int i=0; i<a.size(); i++)
	{
		frame_name.push_back((std::string) a[i]);
	}
	n.getParam("activeSensor",a);
	for(int i=0; i<a.size(); i++)
	{	
		activeSensors.push_back((bool) a[i]);
	}
	n.getParam("min_range",a);
	for(int i=0; i<a.size(); i++)
	{	
		minRange.push_back((double) a[i]);
	}
	n.getParam("max_range",a);
	for(int i=0; i<a.size(); i++)
	{	
		maxRange.push_back((double) a[i]);
	}
	n.getParam("field_of_view",a);
	for(int i=0; i<a.size(); i++)
	{	
		field_of_view.push_back((double) a[i]);
	}
	XmlRpc::XmlRpcValue e;
	n.getParam("translation_x",e);
	XmlRpc::XmlRpcValue f;
	n.getParam("translation_x",f);
	XmlRpc::XmlRpcValue g;
	n.getParam("translation_x",g);
	for(int i=0; i<e.size(); i++)
	{
		std::vector<double> temp;
		temp.push_back((double) e[i]);
		temp.push_back((double) f[i]);
		temp.push_back((double) g[i]);
		position.push_back(temp);
	}
	n.getParam("orientation_yaw",e);
	n.getParam("orientation_pitch",f);
	n.getParam("orientation_roll",g);
	for(int i=0; i<e.size(); i++)
	{
		std::vector<double> temp;
		temp.push_back((double) e[i]);
		temp.push_back((double) f[i]);
		temp.push_back((double) g[i]);
		orientation.push_back(temp);
	}
	for(int i=0; i<a.size(); i++)
	{
		if(activeSensors[i])
		{
			ROS_INFO("sensor nr: %i", i);
			ROS_INFO("radiation type: %i",sensor_type[i]);
			ROS_INFO("min range: %f",minRange[i]);
			ROS_INFO("max range: %f",maxRange[i]);
			ROS_INFO("field of view: %f",field_of_view[i]);
		}

	}

	topicPub_ir = n.advertise<sensor_msgs::Range>("/range_ir",1);
	ROS_INFO("started relayboard _ir wrapper");
	return 0;
}

void SRB_IR_Node::send_IR_State(const cob_relayboard::IRSensors& srsState)
{
	sensor_msgs::Range state;

	for(int i=0; i<(unsigned int) activeSensors.size(); i++)
	{
		if(activeSensors[i])
		{
			//header
			state.header.stamp = ros::Time::now();
			state.header.frame_id = frame_name[i];
			//sensor type:
			state.radiation_type = sensor_type[i];
			state.field_of_view = field_of_view[i];
			state.min_range = minRange[i]; 
			state.max_range = maxRange[i];
			//range:
			state.range = srsState.measurement[i];
			topicPub_ir.publish(state);
		}
	}



}


/**
*/
SRB_IR_Node::SRB_IR_Node () 
{




}

/**
*/
int main (int argc, char** argv)
{
	ros::init(argc, argv, "neo_SRB_IR__node");
	
	SRB_IR_Node  node;
	if(node.init() != 0) return 1;
	ros::spin();
	return 0;
}
