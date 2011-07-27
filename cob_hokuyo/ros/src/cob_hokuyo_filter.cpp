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
 * Date of creation: June 2011
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

// ROS message includes
#include <sensor_msgs/LaserScan.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>


//####################
//#### node class ####
class NodeClass
{
    //
    public:
    int start_left_scan, stop_left_scan, start_right_scan, stop_right_scan;
	int start_tray_filter, stop_tray_filter;
	double tray_filter_min_angle, tray_filter_max_angle;
	bool bFilterTray_;
	      
    ros::NodeHandle nodeHandle;   
    // topics to publish
    ros::Subscriber topicSub_LaserScan_raw;
	ros::Subscriber topicSub_Tray;
    ros::Publisher topicPub_LaserScan;
	ros::Publisher topicPub_LaserScan_self;

    NodeClass()
    {
      // loading config
      nodeHandle.param<int>("start_left_scan", start_left_scan, 0);
      nodeHandle.param<int>("stop_left_scan", stop_left_scan, 248);
	  nodeHandle.param<int>("start_right_scan", start_right_scan, 442);
      nodeHandle.param<int>("stop_right_scan", stop_right_scan, 681);
	  nodeHandle.param<int>("start_tray_filter", start_tray_filter, 442);
      nodeHandle.param<int>("stop_tray_filter", stop_tray_filter, 520);
	  nodeHandle.param<double>("tray_filter_min_angle", tray_filter_min_angle, 0);
      nodeHandle.param<double>("tray_filter_max_angle", tray_filter_max_angle, 1.57);
      // implementation of topics to publish
      topicPub_LaserScan = nodeHandle.advertise<sensor_msgs::LaserScan>("scan_top_filtered", 1);
      topicPub_LaserScan_self = nodeHandle.advertise<sensor_msgs::LaserScan>("scan_top_self_filtered", 1);
      topicSub_LaserScan_raw = nodeHandle.subscribe("scan_top", 1, &NodeClass::scanCallback, this);
      topicSub_Tray = nodeHandle.subscribe("/tray_controller/state", 1, &NodeClass::trayCallback, this);
	  bFilterTray_ = false;
    }


    void trayCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg)
    {
		if(msg->actual.positions[0] > tray_filter_min_angle and msg->actual.positions[0] < tray_filter_max_angle)
			bFilterTray_ = true;
		else
			bFilterTray_ = false;
	}

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
		// create LaserScan message
  		sensor_msgs::LaserScan laserScan;
		laserScan.header.stamp = msg->header.stamp;
            
		// fill message
		laserScan.header.frame_id = msg->header.frame_id;
		laserScan.angle_increment = msg->angle_increment;
		laserScan.range_min = msg->range_min; //TODO read from ini-file/parameter-file
		laserScan.range_max = msg->range_max; //TODO read from ini-file/parameter-file
		laserScan.time_increment = msg->time_increment; //TODO read from ini-file/parameter-file
		
		// rescale scan
		int num_readings = (stop_right_scan - start_left_scan);
		laserScan.angle_min = msg->angle_min; //     first ScanAngle
		laserScan.angle_max = msg->angle_max; // 		last ScanAngle
		laserScan.set_ranges_size(num_readings);
		laserScan.set_intensities_size(0);

		for(int i = 0; i < (stop_left_scan - start_left_scan); i++)
		{
		    	laserScan.ranges[i] = msg->ranges[start_left_scan + i];
		}
		for(int i = stop_left_scan+1; i < start_right_scan; i++)
		{
		    	laserScan.ranges[i] = 0.0;
		}
		for(int i = start_right_scan; i < stop_right_scan; i++)
		{
		    	laserScan.ranges[i] = msg->ranges[i];
		}
		        
		sensor_msgs::LaserScan laserScan_self;
		laserScan_self = laserScan;
		if(bFilterTray_)
		{
			for(int i = start_tray_filter; i < stop_tray_filter; i++)
			{
					laserScan_self.ranges[i] = 0.0;
			}
		}
		// publish message
		topicPub_LaserScan.publish(laserScan);
		topicPub_LaserScan_self.publish(laserScan_self);
      
    }
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
  // initialize ROS, spezify name of node
  ros::init(argc, argv, "hokuyo_filter");

  NodeClass nc;

  ros::spin();
  return 0;
}

