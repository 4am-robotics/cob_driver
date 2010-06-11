/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_forcetorque
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 * Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 *
 * Date of creation: June 2010
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/


#include <stdint.h>
typedef unsigned char uint8_t;
#include <inttypes.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <cob_forcetorque/ForceTorqueCtrl.h>

#include <cob_srvs/Trigger.h>

#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <math.h>
#include <iostream>
#define PI 3.14159265


class ForceTorqueNode
{
public:
  // create a handle for this node, initialize node
  ros::NodeHandle nh_;

  bool init();
  bool srvCallback_Init(cob_srvs::Trigger::Request &req,
			cob_srvs::Trigger::Response &res );
  bool srvCallback_Calibrate(cob_srvs::Trigger::Request &req,
			cob_srvs::Trigger::Response &res );
  void updateFTData();
  void visualizeData(double x, double y, double z);

private:
  // declaration of topics to publish
  ros::Publisher topicPub_ForceData_;
  ros::Publisher topicPub_ForceDataBase_;
  ros::Publisher topicPub_Marker_;

  // service servers
  ros::ServiceServer srvServer_Init_;
  ros::ServiceServer srvServer_Calibrate_;

  tf::TransformListener tflistener;
  tf::StampedTransform transform_ee_base;

  bool m_isInitialized;
  ForceTorqueCtrl ftc;
  std::vector<double> F_avg;
  
};

bool ForceTorqueNode::init()
{
  m_isInitialized = false;
  topicPub_ForceData_ = nh_.advertise<std_msgs::Float32MultiArray>("force_values", 100);
  topicPub_ForceDataBase_ = nh_.advertise<std_msgs::Float32MultiArray>("force_values_base", 100);
  topicPub_Marker_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  srvServer_Init_ = nh_.advertiseService("Init", &ForceTorqueNode::srvCallback_Init, this);
  srvServer_Calibrate_ = nh_.advertiseService("Calibrate", &ForceTorqueNode::srvCallback_Calibrate, this);

  
  return true;
}

bool ForceTorqueNode::srvCallback_Init(cob_srvs::Trigger::Request &req,
		      cob_srvs::Trigger::Response &res )
{
  if(!m_isInitialized)
    {
	ftc.SetFXGain(-1674.08485641479, 25.3936432491561, 3936.02718786968, -26695.2539299392, -3463.73728677908, 32320.8777656041);
	ftc.SetFYGain(-4941.11252317989, 32269.5827812235, 1073.82949467087, -15541.8400780814, 3061.89541712948, -18995.9891819409);
	ftc.SetFZGain(39553.9250733854, -501.940034213822, 40905.2545309848, 85.1095865539103, 38879.4015426067, 541.344775537753);
	ftc.SetTXGain(-57.4775857386444, 225.941430274037, -638.238694389357, -116.780649376712, 645.133934885308, -116.310081348745 );
	ftc.SetTYGain(786.70602313107, -4.36504382717595, -422.360387149734, 180.7428885668, -352.389412256677, -232.293941041101);
	ftc.SetTZGain(60.1009854270179, -400.19573754971, 29.142908672741, -392.119024237625, 70.9306507180567, -478.104759057292);

	ftc.SetCalibMatrix();
	ftc.Init();
	ROS_INFO("FTC initialized");

	//set Calibdata to zero
	F_avg.resize(6);
	F_avg[0] = 0.0;
	F_avg[1] = 0.0;
	F_avg[2] = 0.0;
	F_avg[3] = 0.0;
	F_avg[4] = 0.0;
	F_avg[5] = 0.0;


	m_isInitialized = true;
    }
  return true;
}

bool ForceTorqueNode::srvCallback_Calibrate(cob_srvs::Trigger::Request &req,
		      cob_srvs::Trigger::Response &res )
{
  int measurements = 20;
  if(m_isInitialized)
    {
      F_avg[0] = 0.0;
      F_avg[1] = 0.0;
      F_avg[2] = 0.0;
      F_avg[3] = 0.0;
      F_avg[4] = 0.0;
      F_avg[5] = 0.0;
      for(int i = 0; i < measurements; i++)
	{
	  double Fx, Fy, Fz, Tx, Ty, Tz = 0;
	  ftc.ReadSGData(Fx, Fy, Fz, Tx, Ty, Tz);
	  F_avg[0] += Fx;
	  F_avg[1] += Fy;
	  F_avg[2] += Fz;
	  F_avg[3] += Tx;
	  F_avg[4] += Ty;
	  F_avg[5] += Tz;
	}
      for(int i = 0; i < 6; i++)
	F_avg[i] /= measurements;
      return true;
    }
  else
    return false;
}

void ForceTorqueNode::updateFTData()
{
  if(m_isInitialized)
    {
      double Fx, Fy, Fz, Tx, Ty, Tz = 0;
      
      ftc.ReadSGData(Fx, Fy, Fz, Tx, Ty, Tz);
      std_msgs::Float32MultiArray msg;
      msg.data.push_back(Fx-F_avg[0]);
      msg.data.push_back(Fy-F_avg[1]);
      msg.data.push_back(Fz-F_avg[2]);
      msg.data.push_back(Tx-F_avg[3]);
      msg.data.push_back(Ty-F_avg[4]);
      msg.data.push_back(Tz-F_avg[5]);


      topicPub_ForceData_.publish(msg);
      

      tf::Transform fdata_base;
      tf::Transform fdata;
      fdata.setOrigin(tf::Vector3(Fx-F_avg[0], Fy-F_avg[1], Fz-F_avg[2]));

      try{
        tflistener.lookupTransform("arm_7_link", "base_link", ros::Time::now(), transform_ee_base);
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
      }

      fdata_base = fdata * transform_ee_base;

      std_msgs::Float32MultiArray base_msg;
      base_msg.data.push_back(fdata_base.getOrigin().x());
      base_msg.data.push_back(fdata_base.getOrigin().y());
      base_msg.data.push_back(fdata_base.getOrigin().z());
      base_msg.data.push_back(0.0);
      base_msg.data.push_back(0.0);
      base_msg.data.push_back(0.0);
      topicPub_ForceDataBase_.publish(base_msg);
      visualizeData(fdata_base.getOrigin().x(), fdata_base.getOrigin().y(), fdata_base.getOrigin().z());
    }
}

void ForceTorqueNode::visualizeData(double x, double y, double z)
{
  visualization_msgs::Marker marker;
  uint32_t shape = visualization_msgs::Marker::ARROW;
  marker.header.frame_id = "/arm_7_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.scale.x = x;
  marker.scale.y = y;
  marker.scale.z = z;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  topicPub_Marker_.publish(marker);
}






int main(int argc, char ** argv)
{

  ros::init(argc, argv, "talker");
  ForceTorqueNode ftn;
  ftn.init();
  
  ROS_INFO("ForceTorque Sensor Node running.");
  
  ros::Rate loop_rate(10);
  while(ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
      ftn.updateFTData();
    }
  return 0;
}

