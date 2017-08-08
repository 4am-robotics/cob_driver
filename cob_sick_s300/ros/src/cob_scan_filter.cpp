/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 

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
	ros::Subscriber topicSub_laser_scan_raw;
	ros::Publisher topicPub_laser_scan;

	NodeClass() {
		// loading config
		scan_intervals = loadScanRanges();

		// implementation of topics to publish
		topicPub_laser_scan = nh.advertise<sensor_msgs::LaserScan>("scan_out", 1);
		topicSub_laser_scan_raw = nh.subscribe("scan_in", 1, &NodeClass::scanCallback, this);
	}

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
		//if no filter intervals specified
		if(scan_intervals.size()==0) {
			topicPub_laser_scan.publish(*msg);
			return;
		}


		// use hole received message, later only clear some ranges
		sensor_msgs::LaserScan laser_scan = * msg;


		int start_scan, stop_scan, num_scans;
		num_scans = laser_scan.ranges.size();

		stop_scan = 0;
		for ( unsigned int i=0; i<scan_intervals.size(); i++) {
			std::vector<double> * it = & scan_intervals.at(i);

			if( it->at(1) <= laser_scan.angle_min ) {
				ROS_WARN("Found an interval that lies below min scan range, skip!");
				continue;
			}
			if( it->at(0) >= laser_scan.angle_max ) {
				ROS_WARN("Found an interval that lies beyond max scan range, skip!");
				continue;
			}

			if( it->at(0) <= laser_scan.angle_min ) start_scan = 0;
			else {
				start_scan = (int)( (it->at(0) - laser_scan.angle_min) / laser_scan.angle_increment);
			}

			for(int u = stop_scan; u<start_scan; u++) {
				laser_scan.ranges.at(u) = 0.0; //laser_scan.range_min;
			}

			if( it->at(1) >= laser_scan.angle_max ) stop_scan = num_scans-1;
			else {
				stop_scan = (int)( (it->at(1) - laser_scan.angle_min) / laser_scan.angle_increment);
			}

		}

		for(unsigned int u = stop_scan; u<laser_scan.ranges.size(); u++) {
			laser_scan.ranges.at(u) = 0.0; //laser_scan.range_min;
		}

		// publish message
		topicPub_laser_scan.publish(laser_scan);
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
	std::string scan_intervals_param = "scan_intervals";
	std::vector<std::vector<double> > vd_interval_set;
	std::vector<double> vd_interval;

	//grab the range-list from the parameter server if possible
	XmlRpc::XmlRpcValue intervals_list;
	if(nh.hasParam(scan_intervals_param)){
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
			if(vd_interval.at(0)< -M_PI || vd_interval.at(1)< -M_PI) {
				ROS_WARN("Found a scan interval < -PI, skip!");
				continue;
				//throw std::runtime_error("Found a scan interval < -PI!");
			}
			//basic checking validity
			if(vd_interval.at(0)>M_PI || vd_interval.at(1)>M_PI) {
				ROS_WARN("Found a scan interval > PI, skip!");
				continue;
				//throw std::runtime_error("Found a scan interval > PI!");
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

