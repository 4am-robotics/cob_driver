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
 * ROS package name: cob_platform
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
//#include <string>
//#include <sstream>

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

// ROS service includes
#include <cob_srvs/Trigger.h>

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
	ros::init(argc, argv, "cob3_platform_test");

	// create a handle for this node, initialize node
	ros::NodeHandle n;

    // topics to publish
    ros::Publisher topicPub_CmdVel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        
	// topics to subscribe, callback is called for new messages arriving
    //--
    
    // service servers
    //--
        
    // service clients
    ros::ServiceClient srvClient_Init = n.serviceClient<cob_srvs::Trigger>("Init");
    ros::ServiceClient srvClient_base_Init = n.serviceClient<cob_srvs::Trigger>("base_driver/Init");
    ros::ServiceClient srvClient_ctrl_Init = n.serviceClient<cob_srvs::Trigger>("base_controller/Init");
    ros::ServiceClient srvClient_Stop = n.serviceClient<cob_srvs::Trigger>("Stop");
    ros::ServiceClient srvClient_Shutdown = n.serviceClient<cob_srvs::Trigger>("Shutdown");
    
    // external code
	bool srv_querry = false;
	int srv_execute = 1;
	std::string srv_errorMessage = "no error";
    
    char c;
         
    // main loop
    while(n.ok())
    {
        // process user inputs
        std::cout << "Choose service to test ([s]top, [i]nit, shut[d]own, send[C]ommand, [e]xit): ";
        
        std::cin >> c;

        switch(c)
        {
            case 's':
            {
                //ROS_INFO("querry service [Stop]");
                cob_srvs::Trigger srv;
                srv_querry = srvClient_Stop.call(srv);
                srv_execute = srv.response.success;
                srv_errorMessage = srv.response.errorMessage.data.c_str();
                std::cout << srv.response.errorMessage.data.c_str() << std::endl;
              	break;
            }

            case 'd':
            {
                //ROS_INFO("querry service [Shutdown]");
                cob_srvs::Trigger srv;
                srv_querry = srvClient_Shutdown.call(srv);
                srv_execute = srv.response.success;
                srv_errorMessage = srv.response.errorMessage.data.c_str();
              	break;
            }
            
            case 'i':
            {
            	//ROS_INFO("querry service [Init]");
                cob_srvs::Trigger srv;
                srv_querry = srvClient_Init.call(srv);
                srv_execute = srv.response.success;
                srv_errorMessage = srv.response.errorMessage.data.c_str();
              	break;
            }
            
            case 'x':
            {
            	//ROS_INFO("querry service [Init] for base drive chain");
                cob_srvs::Trigger srv;
                srv_querry = srvClient_base_Init.call(srv);
                srv_execute = srv.response.success;
                srv_errorMessage = srv.response.errorMessage.data.c_str();
              	break;
            }
            
            case 'y':
            {
            	//ROS_INFO("querry service [Init] for base controller");
                cob_srvs::Trigger srv;
                srv_querry = srvClient_ctrl_Init.call(srv);
                srv_execute = srv.response.success;
                srv_errorMessage = srv.response.errorMessage.data.c_str();
              	break;
            }
            
            case 'C':
            {
                // create message
                geometry_msgs::Twist msg;
                
                std::cout << "Choose preset target velocity ([0,1,2,3,4,5,6]): ";
                std::cin >> c;
                if (c == '0')
                {
                    msg.linear.x = 0;
                    msg.linear.y = 0;
                    msg.angular.z = 0;
                }
                else if (c == '1')
                {
                    msg.linear.x = -0.02;
                    msg.linear.y = 0.02;
                    msg.angular.z = 0;
                }
                else if (c == '2')
                {
                    msg.linear.x = -0.02;
                    msg.linear.y = 0;
                    msg.angular.z = 0;
                }
                else if (c == '3')
                {
                    msg.linear.x = -0.02;
                    msg.linear.y = -0.02;
                    msg.angular.z = 0;
                }
                else if (c == '4')
                {
                    msg.linear.x = 0;
                    msg.linear.y = 0.02;
                    msg.angular.z = 0;
                }
                else if (c == '5')
                {
                    msg.linear.x = 0;
                    msg.linear.y = 0;
                    msg.angular.z = 0;
                }
                else if (c == '6')
                {
                    msg.linear.x = 0;
                    msg.linear.y = -0.02;
                    msg.angular.z = 0;
                }
                else if (c == '7')
                {
                    msg.linear.x = 0.02;
                    msg.linear.y = 0.02;
                    msg.angular.z = 0;
                }
                else if (c == '8')
                {
                    msg.linear.x = 0.02;
                    msg.linear.y = 0;
                    msg.angular.z = 0;
                }
                else if (c == '9')
                {
                    msg.linear.x = 0.02;
                    msg.linear.y = -0.02;
                    msg.angular.z = 0;
                }
                else if (c == '+')
                {
                    msg.linear.x = 0;
                    msg.linear.y = 0;
                    msg.angular.z = 0.02;
                }
                else if (c == '-')
                {
                    msg.linear.x = 0;
                    msg.linear.y = 0;
                    msg.angular.z = -0.02;
                }
                else
                {
                    ROS_ERROR("invalid target");
                }
                
                topicPub_CmdVel.publish(msg);
                
                std::cout << std::endl;
                srv_querry = true;
                srv_execute = 0;
            	srv_errorMessage = "no error";
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
