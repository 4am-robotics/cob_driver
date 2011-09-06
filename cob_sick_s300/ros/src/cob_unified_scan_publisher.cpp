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
 * ROS package name: cob_sick_s300
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 * Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 *
 * Date of creation: Jan 2011
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

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>

// ROS message includes
#include <sensor_msgs/LaserScan.h>


//####################
//#### node class ####
class NodeClass
{
    //
    public:
          
		ros::NodeHandle nodeHandle;   
    	// topics to publish
    	message_filters::Subscriber<sensor_msgs::LaserScan> * topicSub_LaserFront;
		message_filters::Subscriber<sensor_msgs::LaserScan> * topicSub_LaserBack;
        message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::LaserScan> *sync;
		tf::TransformListener listener_;
		laser_geometry::LaserProjection projector_;
		ros::Publisher topicPub_LaserUnified;

		tf::StampedTransform transform_scan_front;
	    tf::StampedTransform transform_scan_back;
	  

    NodeClass()
    {
      // loading config
      // implementation of topics to publish
      topicPub_LaserUnified = nodeHandle.advertise<sensor_msgs::LaserScan>("scan_unified", 1);
      topicSub_LaserFront = new message_filters::Subscriber<sensor_msgs::LaserScan>(nodeHandle, "/scan_front", 1);

	  topicSub_LaserBack = new message_filters::Subscriber<sensor_msgs::LaserScan>(nodeHandle, "/scan_rear", 1);
	  sync = new message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::LaserScan>(*topicSub_LaserFront, *topicSub_LaserBack, 10);
	  sync->registerCallback(boost::bind(&NodeClass::scanCallback, this, _1, _2));

	  //listener_.lookupTransform(scan_front->header.frame_id, "base_link",  ros::Time(0), transform_scan_front);
	  //listener_.lookupTransform(scan_rear->header.frame_id, "base_link",  ros::Time(0), transform_scan_back);

    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_front, const sensor_msgs::LaserScan::ConstPtr& scan_rear)
    {
      // create LaserScan message
	  sensor_msgs::LaserScan laserUnified;
	  sensor_msgs::PointCloud cloud_front;
	  //ROS_INFO("Converting front scan to point cloud");
	  listener_.waitForTransform("/base_link", scan_front->header.frame_id, scan_front->header.stamp, ros::Duration(15.0));
	  projector_.transformLaserScanToPointCloud("/base_link",*scan_front, cloud_front, listener_);
	  sensor_msgs::PointCloud cloud_rear;
	  //ROS_INFO("Converting rear scan to point cloud");
	  listener_.waitForTransform("/base_link", scan_rear->header.frame_id, scan_rear->header.stamp, ros::Duration(15.0));
	  projector_.transformLaserScanToPointCloud("/base_link",*scan_rear, cloud_rear, listener_);

	  //ROS_INFO("Creating message header");
	  laserUnified.header = scan_front->header;
      laserUnified.header.frame_id = "base_link";
	  laserUnified.angle_min = -M_PI+0.1;
	  laserUnified.angle_max = M_PI-0.1;
	  laserUnified.angle_increment = M_PI/180.0/2.0;
	  laserUnified.time_increment = 0.0;
	  laserUnified.scan_time = scan_front->scan_time;
	  laserUnified.range_min = scan_front->range_min;
	  laserUnified.range_max = scan_front->range_max;
	  laserUnified.ranges.resize((laserUnified.angle_max - laserUnified.angle_min) / laserUnified.angle_increment);
	  //ROS_INFO("Converting scan from point cloud");
	  for (unsigned int i = 0; i < cloud_front.points.size(); i++)
      {
      	const float &x = cloud_front.points[i].x;
      	const float &y = cloud_front.points[i].y;
      	const float &z = cloud_front.points[i].z;
		//ROS_INFO("Point %f %f %f", x, y, z);
		if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
      	{
			ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
        	continue;
		}
		double angle = atan2(y, x);// + M_PI/2;
      	if (angle < laserUnified.angle_min || angle > laserUnified.angle_max)
      	{
        	ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, laserUnified.angle_min, laserUnified.angle_max);
        	continue;
      	}
		int index = (angle - laserUnified.angle_min) / laserUnified.angle_increment;
		double range_sq = y*y+x*x;
        //printf ("index xyz( %f %f %f) angle %f index %d range %f\n", x, y, z, angle, index, sqrt(range_sq));
        laserUnified.ranges[index] = sqrt(range_sq);
	  }
	  for (unsigned int i = 0; i < cloud_rear.points.size(); i++)
      {
      	const float &x = cloud_rear.points[i].x;
      	const float &y = cloud_rear.points[i].y;
      	const float &z = cloud_rear.points[i].z;
		//ROS_INFO("Point %f %f %f", x, y, z);
		if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
      	{
			ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
        	continue;
		}
		double angle = atan2(y, x); //+ M_PI/2; //TODO: no ideas why but it works
		
      	if (angle < laserUnified.angle_min || angle > laserUnified.angle_max)
      	{
        	ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, laserUnified.angle_min, laserUnified.angle_max);
        	continue;
      	}
		int index = (angle - laserUnified.angle_min) / laserUnified.angle_increment;
		double range_sq = y*y+x*x;
        //printf ("index xyz( %f %f %f) angle %f index %d range %f\n", x, y, z, angle, index, sqrt(range_sq));
        laserUnified.ranges[index] = sqrt(range_sq);
	  }

	  topicPub_LaserUnified.publish(laserUnified);
    }
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
  // initialize ROS, spezify name of node
  ros::init(argc, argv, "scanner_unifier");

  NodeClass nc;

  ros::spin();
  return 0;
}

