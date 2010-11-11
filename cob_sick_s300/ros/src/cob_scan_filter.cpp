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
 * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Jan 2010
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


//####################
//#### node class ####
class NodeClass
{
    //
    public:
    int start_scan, stop_scan;
	      
		ros::NodeHandle nodeHandle;   
    // topics to publish
    ros::Subscriber topicSub_LaserScan_raw;
    ros::Publisher topicPub_LaserScan;

    NodeClass()
    {
      // loading config
      nodeHandle.param<int>("start_scan", start_scan, 115);
      nodeHandle.param<int>("stop_scan", stop_scan, 426);
      // implementation of topics to publish
      topicPub_LaserScan = nodeHandle.advertise<sensor_msgs::LaserScan>("scan_filtered", 1);
      topicSub_LaserScan_raw = nodeHandle.subscribe("scan", 1, &NodeClass::scanCallback, this);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
      // create LaserScan message
      sensor_msgs::LaserScan laserScan;
			laserScan.header.stamp = msg->header.stamp;
            
			// fill message
			laserScan.header.frame_id = msg->header.frame_id;
			laserScan.angle_increment = msg->angle_increment;
			laserScan.range_min = 0.0; //TODO read from ini-file/parameter-file
			laserScan.range_max = 100.0; //TODO read from ini-file/parameter-file
			laserScan.time_increment = msg->time_increment; //TODO read from ini-file/parameter-file
			
			// rescale scan
			int num_readings = stop_scan - start_scan;
			laserScan.angle_min = -135 + laserScan.angle_increment * start_scan; // first ScanAngle
			laserScan.angle_max = -135 + laserScan.angle_increment * start_scan; // last ScanAngle
   		laserScan.set_ranges_size(num_readings);
    	laserScan.set_intensities_size(num_readings);

			for(int i = 0; i < (stop_scan - start_scan); i++)
			{
			    	laserScan.ranges[i] = msg->ranges[start_scan + i];
			    	laserScan.intensities[i] = msg->intensities[start_scan + i];
			}
        
      // publish message
      topicPub_LaserScan.publish(laserScan);
      
    }
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
  // initialize ROS, spezify name of node
  ros::init(argc, argv, "scanner_filter");

  NodeClass nc;

  ros::spin();
  return 0;
}

