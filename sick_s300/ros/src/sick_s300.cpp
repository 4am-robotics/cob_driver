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
 * ROS stack name: cob3_driver
 * ROS package name: sick_s300
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

// ROS service includes
//--

// external includes
#include <sick_s300/ScannerSickS300.h>

//####################
//#### node class ####
class NodeClass
{
    //
    public:
	    // create a handle for this node, initialize node
	    ros::NodeHandle n;
                
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
		int baud;
		bool inverted;
		std::string frame_id;

        // Constructor
        NodeClass()
        {
            // initialize global variables
			n.param<std::string>("port", port, "/dev/ttyUSB0");
			n.param<int>("baud", baud, 500000);
			n.param<bool>("inverted", inverted, false);
			n.param<std::string>("laser_front/frame_id", frame_id, "base_laser_front");

        	// implementation of topics to publish
            topicPub_LaserScan = n.advertise<sensor_msgs::LaserScan>("scan", 1);

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
        	int num_readings = vdDistM.size();
			double laser_frequency = 10; //TODO: read from Ini-file
			
			// create LaserScan Data-Container
        	sensor_msgs::LaserScan laserScan;

			// get time stamp for header
			laserScan.header.stamp = ros::Time::now();
            
			// set scan parameters
//			laserScan.header.frame_id = "base_laser_front"; // TODO read from parameter
			//if(!n.getParam("frame_id", frame_id))
			//	frame_id = "ffff";
			//n.getParam("frame_id", frame_id);
			//ROS_INFO("Laser frame = %s",frame_id.c_str());
			laserScan.header.frame_id = frame_id;
			laserScan.angle_min = vdAngRAD[0]; // first ScanAngle
			laserScan.angle_max = vdAngRAD[num_readings - 1]; // last ScanAngle
			laserScan.angle_increment = vdAngRAD[1] - vdAngRAD[0];
			laserScan.range_min = 0.0; //TODO read from ini-file/parameter-file
			laserScan.range_max = 100.0; //TODO read from ini-file/parameter-file

			laserScan.time_increment = (1 / laser_frequency) / (num_readings); //TODO read from ini-file/parameter-file

   			laserScan.set_ranges_size(num_readings);
    		laserScan.set_intensities_size(num_readings);
			ROS_INFO("LaserScanner Invertiert: %d", inverted);
			for(int i = 0; i < num_readings; i++)
			{
				if(inverted)
				{
			    	laserScan.ranges[i] = vdDistM[num_readings-1-i];
			    	laserScan.intensities[i] = vdIntensAU[num_readings-1-i];
				}
				else
				{
			    	laserScan.ranges[i] = vdDistM[i];
			    	laserScan.intensities[i] = vdIntensAU[i];
				}
			}
        
        	// publish message
            topicPub_LaserScan.publish(laserScan);
        	ROS_DEBUG("published new LaserScan message");
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
	const char pcPort[] = "/dev/ttyUSB1";
//	int iBaudRate = 500000;
	int iBaudRate = nodeClass.baud;
	bool bOpenScan, bRecScan = false;
	bool firstTry = true;
	std::vector<double> vdDistM, vdAngRAD, vdIntensAU;
 
 	while (!bOpenScan)
 	{
 		ROS_INFO("Opening scanner...");
		bOpenScan = SickS300.open(nodeClass.port.c_str(), iBaudRate);
		
		// check, if it is the first try to open scanner
	 	if(firstTry)
		{
			ROS_ERROR("...scanner not available on port %s. Will retry every second.",pcPort);
			firstTry = false;
			sleep(1);
		}
		else
		{
			sleep(1);
		}
	}
	ROS_INFO("...scanner opened successfully on port %s",pcPort);

	// main loop
	ros::Rate loop_rate(5); // Hz
    while(nodeClass.n.ok())
    {
		// read scan
		ROS_DEBUG("Reading scanner...");
		bRecScan = SickS300.getScan(vdDistM, vdAngRAD, vdIntensAU);
		ROS_DEBUG("...scanner read successfully");
    	// publish LaserScan
        if(bRecScan)
        {
		    ROS_DEBUG("...publishing LaserScan message");
            nodeClass.publishLaserScan(vdDistM, vdAngRAD, vdIntensAU);
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

//##################################
//#### function implementations ####
//--
