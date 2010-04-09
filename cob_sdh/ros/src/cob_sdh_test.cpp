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
 * ROS package name: sdh
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
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
//#include <string>
//#include <sstream>
#include <unistd.h>

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <cob_msgs/JointCommand.h>

// ROS service includes
#include <cob_srvs/Init.h>

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
	ros::init(argc, argv, "sdh_test");

	// create a handle for this node, initialize node
	ros::NodeHandle n;

    // topics to publish
    ros::Publisher topicPub_JointCommand = n.advertise<cob_msgs::JointCommand>("joint_commands", 1);
        
	// topics to subscribe, callback is called for new messages arriving
    //--
    
    // service servers
    //--
        
    // service clients
    ros::ServiceClient srvClient_Init = n.serviceClient<cob_srvs::Init>("Init");
    
    // external code
	bool srv_querry = false;
	int srv_execute = 1;
	std::string srv_errorMessage = "no error";
    
    char c;
         
    // main loop
    while(n.ok())
    {
        // process user inputs
        std::cout << "Choose to test ([i]nit, send[C]ommand, [e]xit): ";
        
        std::cin >> c;

        switch(c)
        {
            case 'C':
            {
                // create message
                cob_msgs::JointCommand msg;
                msg.positions.resize(7);
                
                std::cout << "Choose preset target position ([0=parallel, 1=cyl_closed, 2=cyl_open, 3=sher_closed, 4=sher_open, 5=long time test]): ";
                std::cin >> c;
                if (c == '0')
                {
                    msg.positions[0] = 0;
                    msg.positions[1] = 0;
                    msg.positions[2] = 0;
                    msg.positions[3] = 0;
                    msg.positions[4] = 0;
                    msg.positions[5] = 0;
                    msg.positions[6] = 0;
                }
                else if (c == '1')
                {
                    msg.positions[0] = 1;
                    msg.positions[1] = 1;
                    msg.positions[2] = 1;
                    msg.positions[3] = 1;
                    msg.positions[4] = 1;
                    msg.positions[5] = 1;
                    msg.positions[6] = 1;
                }
                else if (c == '2')
                {
                    msg.positions[0] = 2;
                    msg.positions[1] = 2;
                    msg.positions[2] = 2;
                    msg.positions[3] = 2;
                    msg.positions[4] = 2;
                    msg.positions[5] = 2;
                    msg.positions[6] = 2;
                }
                else if (c == '3')
                {
                    msg.positions[0] = 3;
                    msg.positions[1] = 3;
                    msg.positions[2] = 3;
                    msg.positions[3] = 3;
                    msg.positions[4] = 3;
                    msg.positions[5] = 3;
                    msg.positions[6] = 3;
                }
                else if (c == '4')
                {
                    msg.positions[0] = 4;
                    msg.positions[1] = 4;
                    msg.positions[2] = 4;
                    msg.positions[3] = 4;
                    msg.positions[4] = 4;
                    msg.positions[5] = 4;
                    msg.positions[6] = 4;
                }
				else if(c == '5')
				{
					for(;;) {
						msg.positions[0] = 0;
						msg.positions[1] = 0;
						msg.positions[2] = 0;
						msg.positions[3] = 0;
						msg.positions[4] = 0;
						msg.positions[5] = 0;
						msg.positions[6] = 0;
						topicPub_JointCommand.publish(msg);
                        usleep(500000);
						msg.positions[0] = 4;
						msg.positions[1] = 4;
						msg.positions[2] = 4;
						msg.positions[3] = 4;
						msg.positions[4] = 4;
						msg.positions[5] = 4;
						msg.positions[6] = 4;
						topicPub_JointCommand.publish(msg);
                        usleep(500000);
					}
				}
				else
                {
                    ROS_ERROR("invalid target");
                }
                
                topicPub_JointCommand.publish(msg);
                
                std::cout << "ende" << std::endl;
                srv_querry = true;
                srv_execute = 0;
            	srv_errorMessage = "no error";
                break;
            }
            
            case 'i':
            {
            	ROS_INFO("querry service [Init]");
                cob_srvs::Init srv;
                srv_querry = srvClient_Init.call(srv);
                srv_execute = srv.response.success;
                srv_errorMessage = srv.response.errorMessage.data.c_str();
              	break;
            }
            
            case 'e':
            {
                ROS_INFO("exit. Shutting down node");
                std::cout << std::endl;
                return 0;
                break;
            }
            
            default:
            {
                std::cout << "ERROR: invalid input, try again..." << std::endl << std::endl;
            }
        } //switch
        
		if (!srv_querry)
		{
			ROS_ERROR("Failed to call service");
		}
		else
		{
			ROS_INFO("Service call succesfull");
			
			if (srv_execute != 0)
			{
				ROS_ERROR("Service execution failed, errorMessage: %s", srv_errorMessage.c_str());
			}
		}
		
	} //while

    return 0;
} //main
