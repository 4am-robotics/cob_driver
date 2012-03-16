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
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Range.h>

class TF_SRB_US 
{
   public:
	TF_SRB_US();
	~TF_SRB_US();
	int init();
	ros::NodeHandle n;
	ros::Subscriber sub;
	tf::TransformBroadcaster br;
	void rangeSubs(const sensor_msgs::Range::ConstPtr& r);
   private:
	tf::Transform* transforms;
	std::vector<bool> activeSensors;
	std::vector<int> sensor_type;
	std::vector<std::string> frame_name;
	std::vector<double> minRange, maxRange, field_of_view;
	std::vector< std::vector<double> > position, orientation;
	
};

int TF_SRB_US::init()
{
	//get parameters from config file:
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

	//set up transformations:
	transforms = new tf::Transform[activeSensors.size()];
	for(int i=0; i<position.size(); i++)
	{
		transforms[i].setOrigin( tf::Vector3(position[i][0], position[i][1], position[i][2]) );
		transforms[i].setRotation( tf::Quaternion(orientation[i][0], orientation[i][1], orientation[i][2]) );
	}
	//setup broadcaster:
	sub = n.subscribe("/range_us",1,&TF_SRB_US::rangeSubs, this);
	ROS_INFO("started srb us transformation broadcaster");


	return 0;	
}

TF_SRB_US::TF_SRB_US()
{
	transforms = NULL;
}


TF_SRB_US::~TF_SRB_US()
{
	if(transforms) delete[] transforms;

}

void TF_SRB_US::rangeSubs(const sensor_msgs::Range::ConstPtr& r)
{
	for(int i=0; i<position.size(); i++)
	{
		if(r->header.frame_id == frame_name[i])
		{
			br.sendTransform(tf::StampedTransform(transforms[i], ros::Time::now(), "base_link", frame_name[i]));
			break;
		}
	}
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "tf_srb_us");
	TF_SRB_US tf;
	if(tf.init() != 0) return 1;
	ros::spin();	
	return 0;
};
