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
 * Date of creation: Feb 2011
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

// ROS service includes
//--

// external includes
#include <sick-s300/SickS300.hpp>

using namespace brics_oodl;


class NodeClass
{
    //
    public:
	      
		ros::NodeHandle nodeHandle;   
        // topics to publish
        ros::Publisher topicPub_LaserScan;
             
        // global variables
        std::string port;
        int baud, start_scan, stop_scan, scan_id;
        bool inverted;
        std::string frame_id;


        // Constructor
        NodeClass()
        {
			// create a handle for this node, initialize node
	    	ros::NodeHandle private_nh_("~");
            // initialize global variables
			private_nh_.param<std::string>("port", port, "/dev/ttyUSB0");
			private_nh_.param<int>("baud", baud, 500000);
			private_nh_.param<int>("scan_id", scan_id, 7);
			private_nh_.param<bool>("inverted", inverted, false);
			private_nh_.param<std::string>("frame_id", frame_id, "/base_laser_link");
            private_nh_.param<int>("start_scan", start_scan, 0);
            private_nh_.param<int>("stop_scan", stop_scan, 541);

        	// implementation of topics to publish
            topicPub_LaserScan = nodeHandle.advertise<sensor_msgs::LaserScan>("scan", 1);

        
        }
        
        // Destructor
        ~NodeClass() 
        {
        }

	void publishLaserScan(std::vector<double> vdDistM, std::vector<double> vdAngRAD, std::vector<double> vdIntensAU)
        {
        	// fill message
        	int num_readings = vdDistM.size(); // initialize with max scan size
      		start_scan = 0;
            stop_scan = vdDistM.size();
			double laser_frequency = 10; //TODO: read from Ini-file
			
			// create LaserScan message
        	sensor_msgs::LaserScan laserScan;
			laserScan.header.stamp = ros::Time::now();
            
			// fill message
			laserScan.header.frame_id = frame_id;
			laserScan.angle_increment = vdAngRAD[start_scan + 1] - vdAngRAD[start_scan];
			laserScan.range_min = -135.0/180.0*3.14;//0.0; //TODO read from ini-file/parameter-file
			laserScan.range_max = 135.0/180.0*3.14;//100.0; //TODO read from ini-file/parameter-file
			laserScan.time_increment = (1 / laser_frequency) / (vdDistM.size()); //TODO read from ini-file/parameter-file
			
			// rescale scan
			num_readings = stop_scan - start_scan;
			laserScan.angle_min = vdAngRAD[start_scan]; // first ScanAngle
			laserScan.angle_max = vdAngRAD[stop_scan - 1]; // last ScanAngle
   			laserScan.set_ranges_size(num_readings);
    		laserScan.set_intensities_size(num_readings);

			// check for inverted laser
		//	inverted = true; //true; // TODO remove hardcoded parameter
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
	


    LaserScannerConfiguration config;
    config.devicePath = nodeClass.port.c_str(); // Device path of the Sick
    //config.baud = nodeClass.baud;

    SickS300 scanner;
    Errors errors;
	
	int iScanId = nodeClass.scan_id;
	bool bOpenScan = false, bRecScan = false;
	bool firstTry = true;
	std::vector<double> vdDistM, vdAngRAD, vdIntensAU;
 
 	while (!bOpenScan)
 	{
 		ROS_INFO("Opening scanner... (port:%s)",nodeClass.port.c_str());
 		Errors errors;

        if (!scanner.setConfiguration(config, errors)) {
            errors.printErrorsToConsole();
            return -1;
        }
		//Old code bOpenScan = SickS300.open(nodeClass.port.c_str(), iBaudRate, iScanId);
		
		// check, if it is the first try to open scanner
	 	if(!bOpenScan)
		{
			ROS_ERROR("...scanner not available on port %s. Will retry every second.",nodeClass.port.c_str());
			firstTry = false;
		}
		sleep(1);
	}
	
	ROS_INFO("...scanner opened successfully on port %s",nodeClass.port.c_str());

	ros::Rate loop_rate(5); // Hz
	LaserScannerData scanData;
    while(nodeClass.nodeHandle.ok())
    {
		ROS_DEBUG("Reading scanner...");		
		bRecScan= scanner.getData(scanData, errors);
		//Old code bRecScan = SickS300.getScan(vdDistM, vdAngRAD, vdIntensAU);
	    ROS_DEBUG("...read %d points from scanner successfully",vdDistM.size());
    	
    	if(bRecScan)
        {
		    ROS_DEBUG("...publishing LaserScan message");
            //nodeClass.publishLaserScan(vdDistM, vdAngRAD, vdIntensAU);
        }
        else
        {
		    ROS_WARN("...no Scan available");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


