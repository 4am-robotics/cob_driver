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
#include <diagnostic_msgs/DiagnosticArray.h>

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
	      
		ros::NodeHandle nodeHandle;   
        // topics to publish
        ros::Publisher topicPub_LaserScan;
        ros::Publisher topicPub_Diagnostic_;

	    // topics to subscribe, callback is called for new messages arriving
		//--
        
        // service servers
        //--
            
        // service clients
        //--
        
        // global variables
        std::string port;
        int baud, start_scan, stop_scan, scan_id;
        bool inverted;
        std::string frame_id;


        // Constructor
        NodeClass()
        {
			// create a handle for this node, initialize node
	    	nodeHandle = ros::NodeHandle("~");
            // initialize global variables
			nodeHandle.param<std::string>("port", port, "/dev/ttyUSB0");
			nodeHandle.param<int>("baud", baud, 500000);
			nodeHandle.param<int>("scan_id", scan_id, 7);
			nodeHandle.param<bool>("inverted", inverted, false);
			nodeHandle.param<std::string>("frame_id", frame_id, "/base_laser_link");
            nodeHandle.param<int>("start_scan", start_scan, 0);
            nodeHandle.param<int>("stop_scan", stop_scan, 541);

        	// implementation of topics to publish
            topicPub_LaserScan = nodeHandle.advertise<sensor_msgs::LaserScan>("scan", 1);
	    topicPub_Diagnostic_ = nodeHandle.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

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
	    diagnostic_msgs::DiagnosticArray diagnostics;
	    diagnostics.status.resize(1);
	    diagnostics.status[0].level = 0;
	    diagnostics.status[0].name = nodeHandle.getNamespace();
	    diagnostics.status[0].message = "sick scanner running";
	    topicPub_Diagnostic_.publish(diagnostics); 

        }
  void publishError(std::string error_str)
  {
    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostics.status.resize(1);
    diagnostics.status[0].level = 2;
    diagnostics.status[0].name = nodeHandle.getNamespace();
    diagnostics.status[0].message = error_str;
    topicPub_Diagnostic_.publish(diagnostics);     
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

	//char *pcPort = new char();
//	const char pcPort[] = "/dev/ttyUSB1"; //TODO replace with parameter port
//	const char pcPort[] = nodeClass.port;
//	int iBaudRate = 500000;
	int iBaudRate = nodeClass.baud;
	int iScanId = nodeClass.scan_id;
	bool bOpenScan = false, bRecScan = false;
	bool firstTry = true;
	std::vector<double> vdDistM, vdAngRAD, vdIntensAU;
 
 	while (!bOpenScan)
 	{
 		ROS_INFO("Opening scanner... (port:%s)",nodeClass.port.c_str());
		bOpenScan = SickS300.open(nodeClass.port.c_str(), iBaudRate, iScanId);
		
		// check, if it is the first try to open scanner
	 	if(!bOpenScan)
		{
			ROS_ERROR("...scanner not available on port %s. Will retry every second.",nodeClass.port.c_str());
			nodeClass.publishError("...scanner not available on port");
			firstTry = false;
		}
		sleep(1); // wait for scann to get ready if successfull, or wait befor retrying
	}
	ROS_INFO("...scanner opened successfully on port %s",nodeClass.port.c_str());

	// main loop
	ros::Rate loop_rate(5); // Hz
    while(nodeClass.nodeHandle.ok())
    {
		// read scan
		ROS_DEBUG("Reading scanner...");
		bRecScan = SickS300.getScan(vdDistM, vdAngRAD, vdIntensAU);
		ROS_DEBUG("...read %d points from scanner successfully",vdDistM.size());
    	// publish LaserScan
        if(bRecScan)
        {
		    ROS_DEBUG("...publishing LaserScan message");
            nodeClass.publishLaserScan(vdDistM, vdAngRAD, vdIntensAU);
        }
        else
        {
		    ROS_WARN("...no Scan available");
		    nodeClass.publishError("...no Scan available");
        }

        // sleep and waiting for messages, callbacks    
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

//##################################
//#### function implementations ####
//--
