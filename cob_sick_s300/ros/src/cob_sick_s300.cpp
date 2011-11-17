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
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/LaserScan.h>

// ROS service includes
//--

// external includes
#include <cob_sick_s300/ScannerSickS300.h>

//####################
//#### node class ####
class NodeClass
{
	//
	public:
		  
		ros::NodeHandle nh;   
		// topics to publish
		ros::Publisher topicPub_LaserScan;
		
		// topics to subscribe, callback is called for new messages arriving
		//--
		
		// service servers
		//--
			
		// service clients
		//--
		
		// global variables
		std::string port;
		int baud, scan_id;
		bool inverted;
		double laser_frequency;
		std::string frame_id;
		ros::Time syncedROSTime;
		unsigned int syncedSICKStamp;
		bool syncedTimeReady;

		// Constructor
		NodeClass()
		{
			// create a handle for this node, initialize node
			nh = ros::NodeHandle("~");
			
			if(!nh.hasParam("port")) ROS_WARN("Used default parameter for port");
			nh.param("port", port, std::string("/dev/ttyUSB0"));
			
			if(!nh.hasParam("baud")) ROS_WARN("Used default parameter for baud");
			nh.param("baud", baud, 500000);
			
			if(!nh.hasParam("scan_id")) ROS_WARN("Used default parameter for scan_id");
			nh.param("scan_id", scan_id, 7);
			
			if(!nh.hasParam("inverted")) ROS_WARN("Used default parameter for inverted");
			nh.param("inverted", inverted, false);
			
			if(!nh.hasParam("frame_id")) ROS_WARN("Used default parameter for frame_id");
			nh.param("frame_id", frame_id, std::string("/base_laser_link"));
			
			if(!nh.hasParam("laser_frequency")) ROS_WARN("Used default parameter for laser_frequency");
			nh.param("laser_frequency", laser_frequency, 1.0/0.040); //SICK-docu says S300 scans every 40ms //10;
			
			syncedSICKStamp = 0;
			syncedROSTime = ros::Time::now();
			syncedTimeReady = false;

			// implementation of topics to publish
			topicPub_LaserScan = nh.advertise<sensor_msgs::LaserScan>("scan", 1);

			// implementation of topics to subscribe
			//--
			
			// implementation of service servers
			//--
		}
		
		// Destructor
		~NodeClass() 
		{
		}

		// topic callback functions 
		// function will be called when a new message arrives on a topic
		//--

		// service callback functions
		// function will be called when a service is querried
		//--
		
		// other function declarations
	void publishLaserScan(std::vector<double> vdDistM, std::vector<double> vdAngRAD, std::vector<double> vdIntensAU, unsigned int iSickTimeStamp, unsigned int iSickNow)
		{
			// fill message
			int start_scan, stop_scan;
			int num_readings = vdDistM.size(); // initialize with max scan size
	  		start_scan = 0;
			stop_scan = vdDistM.size();
			
			//Sync handling: find out exact scan time by using the syncTime-syncStamp pair:
			// Timestamp: "This counter is internally incremented at each scan, i.e. every 40 ms (S300)"
			if(iSickNow != 0) {
				syncedROSTime = ros::Time::now(); //TODO: -1.0/laser_frequency * 2.0? (when has the timestamp actually been set?)
				syncedSICKStamp = iSickNow;
				syncedTimeReady = true;
				
				ROS_DEBUG("Got iSickNow: %d", syncedSICKStamp);
			}
			
			// create LaserScan message
			sensor_msgs::LaserScan laserScan;
			if(syncedTimeReady) {
				double timeDiff = (int)(iSickTimeStamp - syncedSICKStamp) / laser_frequency;
				laserScan.header.stamp = syncedROSTime + ros::Duration(timeDiff);
				
				ROS_DEBUG("time-diff %f",timeDiff);
				ROS_DEBUG("stamp-diff %d",(int)(iSickTimeStamp - syncedSICKStamp));
				ROS_DEBUG("Stamp = %f",laserScan.header.stamp.toSec());
				ROS_DEBUG("NOW - Stamp = %f",(ros::Time::now() - laserScan.header.stamp).toSec());
			} else {
				laserScan.header.stamp = ros::Time::now();
			}
			
			
			// fill message
			laserScan.header.frame_id = frame_id;
			laserScan.angle_increment = vdAngRAD[start_scan + 1] - vdAngRAD[start_scan];
			laserScan.range_min = 0.0;
			laserScan.range_max = 100.0;
			laserScan.time_increment = (1 / laser_frequency) / (vdDistM.size()); //TODO: time increment now meaningless, negative value allowed??
			
			// rescale scan
			num_readings = vdDistM.size();
			laserScan.angle_min = vdAngRAD[start_scan]; // first ScanAngle
			laserScan.angle_max = vdAngRAD[stop_scan - 1]; // last ScanAngle
   			laserScan.ranges.resize(num_readings);
			laserScan.intensities.resize(num_readings);

			// check for inverted laser
			for(int i = 0; i < (stop_scan - start_scan); i++)
			{
				if(inverted)
				{
					laserScan.ranges[i] = vdDistM[stop_scan-1-i];
					laserScan.intensities[i] = vdIntensAU[stop_scan-1-i];
				}
				else
				{
					laserScan.ranges[i] = vdDistM[start_scan + i];
					laserScan.intensities[i] = vdIntensAU[start_scan + i];
				}
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
	ros::init(argc, argv, "sick_s300");
	
	NodeClass nodeClass;
	ScannerSickS300 SickS300;

	int iBaudRate = nodeClass.baud;
	int iScanId = nodeClass.scan_id;
	bool bOpenScan = false, bRecScan = false;
	bool firstTry = true;
	
	unsigned int iSickTimeStamp, iSickNow;
	std::vector<double> vdDistM, vdAngRAD, vdIntensAU;
		
 	while (!bOpenScan)
 	{
 		ROS_INFO("Opening scanner... (port:%s)",nodeClass.port.c_str());
		bOpenScan = SickS300.open(nodeClass.port.c_str(), iBaudRate, iScanId);
		
		// check, if it is the first try to open scanner
	 	if(!bOpenScan)
		{
			ROS_ERROR("...scanner not available on port %s. Will retry every second.",nodeClass.port.c_str());
			firstTry = false;
		}
		sleep(1); // wait for scan to get ready if successfull, or wait befor retrying
	}
	ROS_INFO("...scanner opened successfully on port %s",nodeClass.port.c_str());
	
	// main loop
	ros::Rate loop_rate(5); // Hz
	while(nodeClass.nh.ok())
	{
		// read scan
		ROS_DEBUG("Reading scanner...");
		bRecScan = SickS300.getScan(vdDistM, vdAngRAD, vdIntensAU, iSickTimeStamp, iSickNow);
		ROS_DEBUG("...read %d points from scanner successfully",(int)vdDistM.size());
		// publish LaserScan
		if(bRecScan)
		{
			ROS_DEBUG("...publishing LaserScan message");
			nodeClass.publishLaserScan(vdDistM, vdAngRAD, vdIntensAU, iSickTimeStamp, iSickNow);
		}
		else
		{
			ROS_WARN("...no Scan available");
		}

		// sleep and waiting for messages, callbacks	
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
