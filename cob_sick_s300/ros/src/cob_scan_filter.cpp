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
 *	 * Redistributions of source code must retain the above copyright
 *	   notice, this list of conditions and the following disclaimer.
 *	 * Redistributions in binary form must reproduce the above copyright
 *	   notice, this list of conditions and the following disclaimer in the
 *	   documentation and/or other materials provided with the distribution.
 *	 * Neither the name of the Fraunhofer Institute for Manufacturing 
 *	   Engineering and Automation (IPA) nor the names of its
 *	   contributors may be used to endorse or promote products derived from
 *	   this software without specific prior written permission.
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
#include <vector>
#include <algorithm>

// ROS includes
#include <ros/ros.h>
#include <XmlRpc.h>

// ROS message includes
#include <sensor_msgs/LaserScan.h>


//####################
//#### node class ####
class NodeClass
{
public:
	std::vector<std::vector<double> > scan_intervals;
		  
	ros::NodeHandle nh;   
	// topics to publish
	ros::Subscriber topicSub_LaserScan_raw;
	ros::Publisher topicPub_LaserScan;

	NodeClass() {
		// loading config
		loadScanRanges();	
		
		// implementation of topics to publish
		topicPub_LaserScan = nh.advertise<sensor_msgs::LaserScan>("scan_filtered", 1);
		topicSub_LaserScan_raw = nh.subscribe("scan", 1, &NodeClass::scanCallback, this);
	}

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
		//if no filter intervals specified
		if(scan_intervals.size()==0) {
			topicPub_LaserScan.publish(*msg);
			return;
		}
		
		
		// create LaserScan message
		sensor_msgs::LaserScan laserScan;
		laserScan.header = msg->header;
		laserScan.angle_increment = msg->angle_increment;
		laserScan.range_min = msg->range_min; //0.0 TODO read from ini-file/parameter-file / set correct in sick_s300
		laserScan.range_max = msg->range_max; //100.0 TODO read from ini-file/parameter-file / set correct in sick_s300
		laserScan.time_increment = msg->time_increment;
		
		//check, wether last interval is bigger than scan:
		while( scan_intervals.back().at(0) >= msg->ranges.size()-1 ) {
				scan_intervals.pop_back(); //if begin of last interval >= total scan points delete it
				ROS_WARN("scan filter: received scan message, that is smaller than specified interval");
				if(scan_intervals.size()==0) {
					topicPub_LaserScan.publish(*msg);
					return;
				}
		}

		//implementation for unused ranges = VAL
		int start_scan, stop_scan;
		start_scan = scan_intervals.front().at(0);
		stop_scan = scan_intervals.back().at(1);
		
		int num_readings = start_scan - stop_scan +1;
		laserScan.angle_min = (-135.0/180.0*3.14) + laserScan.angle_increment * start_scan; // first ScanAngle
		laserScan.angle_max = (-135.0/180.0*3.14) + laserScan.angle_increment * stop_scan; // last ScanAngle
		laserScan.ranges.resize(num_readings);
		laserScan.intensities.resize(num_readings);
		
		std::vector<bool> use_current_element;
		for(unsigned int u=0; u<scan_intervals.size()-1; u++) {
			use_current_element.insert(use_current_element.end(), true, scan_intervals.at(u).at(1) - scan_intervals.at(u).at(0) +1);
			use_current_element.insert(use_current_element.end(), false, scan_intervals.at(u+1).at(0) - scan_intervals.at(u).at(1) -1);
		}
		//handle last interval separately because there is no following gap:
		use_current_element.insert(use_current_element.end(), true, scan_intervals.back().at(1) - scan_intervals.back().at(0) +1);
		
		if((int)use_current_element.size() != num_readings) ROS_WARN("Vector size problem, rest is cut off..");

		for(int i=0; i<num_readings; i++) {
			if(use_current_element.at(i)) {
				laserScan.ranges[i] = msg->ranges[start_scan + i];
				laserScan.intensities[i] = msg->intensities[start_scan + i];
			} else {
				laserScan.ranges[i] = 0.0; //filtered out
				laserScan.intensities[i] = 0.0; //filtered out
			}
		}
		
		// publish message
		topicPub_LaserScan.publish(laserScan);
	}
	
	std::vector<std::vector<double> > loadScanRanges();
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

bool compareIntervals(std::vector<double> a, std::vector<double> b) {
	return a.at(0) < b.at(0);
}

std::vector<std::vector<double> > NodeClass::loadScanRanges() {
	std::string scan_intervals_param;
	std::vector<std::vector<double> > vd_interval_set;
	std::vector<double> vd_interval;

	//grab the range-list from the parameter server if possible
	XmlRpc::XmlRpcValue intervals_list;
	if(nh.searchParam("scan_intervals", scan_intervals_param)){
		nh.getParam(scan_intervals_param, intervals_list);
		//make sure we have a list of lists
		if(!(intervals_list.getType() == XmlRpc::XmlRpcValue::TypeArray)){
			ROS_FATAL("The scan intervals must be specified as a list of lists [[x1, y1], [x2, y2], ..., [xn, yn]]");
			throw std::runtime_error("The scan intervals must be specified as a list of lists [[x1, y1], [x2, y2], ..., [xn, yn]]");
		}
		
		for(int i = 0; i < intervals_list.size(); ++i){
			vd_interval.clear();
		
			//make sure we have a list of lists of size 2
			XmlRpc::XmlRpcValue interval = intervals_list[i];
			if(!(interval.getType() == XmlRpc::XmlRpcValue::TypeArray && interval.size() == 2)){
				ROS_FATAL("The scan intervals must be specified as a list of lists [[x1, y1], [x2, y2], ..., [xn, yn]]");
				throw std::runtime_error("The scan intervals must be specified as a list of lists [[x1, y1], [x2, y2], ..., [xn, yn]]");
			}

			//make sure that the value we're looking at is either a double or an int
			if(!(interval[0].getType() == XmlRpc::XmlRpcValue::TypeInt || interval[0].getType() == XmlRpc::XmlRpcValue::TypeDouble)){
				ROS_FATAL("Values in the scan intervals specification must be numbers");
				throw std::runtime_error("Values in the scan intervals specification must be numbers");
			}
			vd_interval.push_back( interval[0].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(interval[0]) : (double)(interval[0]) );

			//make sure that the value we're looking at is either a double or an int
			if(!(interval[1].getType() == XmlRpc::XmlRpcValue::TypeInt || interval[1].getType() == XmlRpc::XmlRpcValue::TypeDouble)){
				ROS_FATAL("Values in the scan intervals specification must be numbers");
				throw std::runtime_error("Values in the scan intervals specification must be numbers");
			}
			vd_interval.push_back( interval[1].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(interval[1]) : (double)(interval[1]) );
			
			//basic checking validity
			if(vd_interval.at(0)<0 || vd_interval.at(1)<0) {
				ROS_FATAL("Found a negative scan interval!");
				throw std::runtime_error("Found a negative scan interval!");
			}
			if(vd_interval.at(0) >= vd_interval.at(1)) {
				ROS_WARN("Found a scan interval with i1 > i2, switched order!");
				vd_interval[1] = vd_interval[0];
				vd_interval[0] = ( interval[1].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(interval[1]) : (double)(interval[1]) );
			}
		
			vd_interval_set.push_back(vd_interval);
		}
	} else ROS_WARN("Scan filter has not found any scan interval parameters.");
	
	//now we want to sort the intervals and check for overlapping
	sort(vd_interval_set.begin(), vd_interval_set.end(), compareIntervals);
	
	for(unsigned int i = 0; i<vd_interval_set.size(); i++) {
		for(unsigned int u = i+1; u<vd_interval_set.size(); u++) {
			if( vd_interval_set.at(i).at(1) > vd_interval_set.at(u).at(0)) {
				ROS_FATAL("The scan intervals you specified are overlapping!");
				throw std::runtime_error("The scan intervals you specified are overlapping!");
			}
		}
	}
	
	/* DEBUG out:
	for(unsigned int i = 0; i<vd_interval_set.size(); i++) {
		std::cout << "Interval " << i << " is " << vd_interval_set.at(i).at(0) << " | " << vd_interval_set.at(i).at(1) << std::endl;
	} */
	
	return vd_interval_set;
}

