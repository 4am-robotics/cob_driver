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
 * ROS package name: cob_drive_identification
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Philipp Koehler
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Mar 2010
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
// Headers provided by cob-packages which should be avoided/removed

// ROS includes
#include <ros/ros.h>
#include <stdio.h>

// ROS message includes
//#include <sensor_msgs/JointState.h>

// ROS service includes
//#include <std_srvs/Empty.h>

// external includes
//--

//##########################
//#### global variables ####
//--

//##################################
//#### topic callback functions ####
// function will be called when a new message arrives on a topic
//--

//####################################
//#### service callback functions ####
// function will be called when a service is querried
//--

float convertBinaryToFloat(unsigned int iBinaryRepresentation) {
	//Converting binary-numbers to 32bit float values according to IEEE 754 see http://de.wikipedia.org/wiki/IEEE_754
	int iSign;
	int iExponent;
	unsigned int iMantissa;
	float iNumMantissa = 0.0f;
	
	std::cout << iBinaryRepresentation << std::endl;


	if((iBinaryRepresentation & (1 << 31)) == 0) //first bit is sign bit: 0 = +, 1 = -
		iSign = 1;
	else
		iSign = -1;

	iExponent = ((iBinaryRepresentation >> 23) & 0xFF) - 127; //take away Bias(127) for positive and negative exponents
	std::cout << "Exp: " << iExponent << std::endl;
	
	iMantissa = (iBinaryRepresentation & 0x7FFFFF); //only keep mantissa part of binary number
	std::cout << iMantissa << std::endl;

	iNumMantissa = 1.0f;
	
	for(int i=1; i<=23; i++) { //calculate decimal places (convert binary mantissa to decimal number
		if((iMantissa & (1 << (23-i))) > 0) {
			iNumMantissa = iNumMantissa + pow(2,(-1)*i);
		}
	}
	std::cout << iNumMantissa << std::endl;
	
	return iSign * pow(2,iExponent) * iNumMantissa;
}

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	std::cout << convertBinaryToFloat(0x5D8929CC) << std::endl;

	
	/*
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "node_name");

	// create a handle for this node, initialize node
	ros::NodeHandle n;
	
    // topics to publish
    ros::Publisher topicPub_JointStateCmd;
    topicPub_JointStateCmd = n.advertise<sensor_msgs::JointState>("JointStateCmd", 1);
    

	// topics to subscribe, callback is called for new messages arriving
    //--
    
    // service servers
    //--
        
    // service clients
    ros::ServiceClient srvClient_GetJointState<cob3_srvs::GetJointState>("GetJointState");
    ros::ServiceClient srvClient_InitPltf<cob3_srvs::Switch>("Init");
    ros::ServiceClient srcClient_ShutdownPltf<cob3_srvs::Switch>("Shutdown");
    
    // external code
  	//--
  	
	
    
    //start identification process
    srvClient_InitPltf.call(cob3_srvs::Switch data);
    if(data.response.success != true) {
        ROS_ERROR("Failed to initialize Platform with message: %s", data.response.errorMessage);
        return 0;
    }

    


    // main loop
    //ros::Rate loop_rate(10); // Hz
	while (n.ok())
	{
        // create message
        //--
        
        // publish message
        //--

        // sleep and waiting for messages, callbacks    
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0; */
}
