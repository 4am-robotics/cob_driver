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
#include <cob3_utilities/IniFile.h>


// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/JointState.h>

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

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
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
    
    return 0;
}
