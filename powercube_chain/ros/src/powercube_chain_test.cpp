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
 * ROS package name: powercube_chain
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
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
//#include <cob_msgs/JointCommand.h>
#include <trajectory_msgs/JointTrajectory.h>

// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>

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
	ros::init(argc, argv, "powercube_chain_test");

	// create a handle for this node, initialize node
	ros::NodeHandle n;

    // topics to publish
    ros::Publisher topicPub_JointCommand = n.advertise<trajectory_msgs::JointTrajectory>("command", 1);
        
	// topics to subscribe, callback is called for new messages arriving
    //--
    
    // service servers
    //--
        
    // service clients
    ros::ServiceClient srvClient_Init = n.serviceClient<cob_srvs::Trigger>("Init");
    ros::ServiceClient srvClient_Stop = n.serviceClient<cob_srvs::Trigger>("Stop");
    ros::ServiceClient srvClient_Recover = n.serviceClient<cob_srvs::Trigger>("Recover");
    ros::ServiceClient srvClient_SetOperationMode = n.serviceClient<cob_srvs::SetOperationMode>("SetOperationMode");
    
    // external code
	bool srv_querry = false;
	int srv_execute = 1;
	std::string srv_errorMessage = "no error";
    
    char c;
         
    // main loop
    while(n.ok())
    {
        // process user inputs
        std::cout << "Choose service to test ([s]top, [i]nit, [r]ecover, setOperation[M]ode, send[C]ommand, [e]xit): ";
        
        
        std::cin >> c;

        switch(c)
        {
            case 's':
            {
                cob_srvs::Trigger srv;
                srv_querry = srvClient_Stop.call(srv);
                srv_execute = srv.response.success;
                srv_errorMessage = srv.response.errorMessage.data.c_str();
              	break;
            }
            
            case 'i':
            {
                cob_srvs::Trigger srv;
                srv_querry = srvClient_Init.call(srv);
                srv_execute = srv.response.success;
                srv_errorMessage = srv.response.errorMessage.data.c_str();
              	break;
            }
            
            case 'r':
            {
                cob_srvs::Trigger srv;
                srv_querry = srvClient_Recover.call(srv);
                srv_execute = srv.response.success;
                srv_errorMessage = srv.response.errorMessage.data.c_str();
              	break;
            }
            
            case 'M':
            {
                cob_srvs::SetOperationMode srv;
                
                std::cout << "Choose operation mode ([v] = velocity controll, [p] = position controll): ";
                std::cin >> c;
                if (c == 'v')
                {
                    srv.request.operationMode.data = "velocity";
                }
                else if (c == 'p')
                {
                    srv.request.operationMode.data = "position";
                }
                else
                {
                    srv.request.operationMode.data = "none";
                }
                ROS_INFO("changing operation mode to: %s controll", srv.request.operationMode.data.c_str());
                
                //ROS_INFO("querry service [cob3/arm/SetOperationMode]");
                srv_querry = srvClient_SetOperationMode.call(srv);
                srv_execute = srv.response.success;
                srv_errorMessage = srv.response.errorMessage.data.c_str();
                break;
            }
            
            case 'C':
            {
                std::cout << "Choose preset target positions/velocities ([0] = , [1] = , [2] = ): ";
                std::cin >> c;
                
                int DOF = 4;
                trajectory_msgs::JointTrajectory msg;
                msg.header.stamp = ros::Time::now();
                msg.points.resize(3); //TODO hardcoded to two points, remove to accept multiple points of trajectory_msgs
                msg.points[0].positions.resize(DOF);
                msg.points[0].velocities.resize(DOF);
                msg.points[1].positions.resize(DOF);
                msg.points[1].velocities.resize(DOF);
                msg.points[2].positions.resize(DOF);
                msg.points[2].velocities.resize(DOF);
                
                if (c == '0')
                {
		            msg.points[0].positions[0] = 0;
		            msg.points[0].positions[1] = 0;
		            msg.points[0].positions[2] = 0;
		            msg.points[0].positions[3] = 0;
		            //msg.points[0].positions[4] = 0;
		            //msg.points[0].positions[5] = 0;
		            //msg.points[0].positions[6] = 0;
		            
		            msg.points[0].velocities[0] = 0;
		            msg.points[0].velocities[1] = 0;
		            msg.points[0].velocities[2] = 0;
		            msg.points[0].velocities[3] = 0;
		            //msg.points[0].velocities[4] = 0;
		            //msg.points[0].velocities[5] = 0;
		            //msg.points[0].velocities[6] = 0;
                }
                else if (c == '1')
                {
                    msg.points[0].positions[0] = 0.1;
		            msg.points[0].positions[1] = 0.1;
		            msg.points[0].positions[2] = 0.1;
		            msg.points[0].positions[3] = 0.1;
		            //msg.points[0].positions[4] = 0.1;
		            //msg.points[0].positions[5] = 0.1;
		            //msg.points[0].positions[6] = 0.1;
		            
		            msg.points[0].velocities[0] = 0.1;
		            msg.points[0].velocities[1] = 0.1;
		            msg.points[0].velocities[2] = 0.1;
		            msg.points[0].velocities[3] = 0.1;
		            //msg.points[0].velocities[4] = 0.1;
		            //msg.points[0].velocities[5] = 0.1;
		            //msg.points[0].velocities[6] = 0.1;
                }
                else if (c == '2')
                {
                    msg.points[0].positions[0] = 0.2;
		            msg.points[0].positions[1] = 0.2;
		            msg.points[0].positions[2] = 0.2;
		            msg.points[0].positions[3] = 0.2;
		            //msg.points[0].positions[4] = 0.2;
		            //msg.points[0].positions[5] = 0.2;
		            //msg.points[0].positions[6] = 0.2;
		            
		            msg.points[0].velocities[0] = 0.2;
		            msg.points[0].velocities[1] = 0.2;
		            msg.points[0].velocities[2] = 0.2;
		            msg.points[0].velocities[3] = 0.2;
		            //msg.points[0].velocities[4] = 0.2;
		            //msg.points[0].velocities[5] = 0.2;
		            //msg.points[0].velocities[6] = 0.2;
                }
                else if (c == '3')
                {
                    msg.points[0].positions[0] = 0.0;
		            msg.points[0].positions[1] = 0.0;
		            msg.points[0].positions[2] = 0.0;
		            msg.points[0].positions[3] = 0.2;
		            //msg.points[0].positions[4] = 0.2;
		            //msg.points[0].positions[5] = 0.2;
		            //msg.points[0].positions[6] = 0.2;
		            
		            msg.points[0].velocities[0] = 0.0;
		            msg.points[0].velocities[1] = 0.0;
		            msg.points[0].velocities[2] = 0.0;
		            msg.points[0].velocities[3] = 0.0;
		            //msg.points[0].velocities[4] = 0.2;
		            //msg.points[0].velocities[5] = 0.2;
		            //msg.points[0].velocities[6] = 0.2;
                }
				else if (c == 't') // trajectory
                {
                	// point 1
                    msg.points[0].positions[0] = 0.2;
		            msg.points[0].positions[1] = 0.0;
		            msg.points[0].positions[2] = 0.0;
		            msg.points[0].positions[3] = 0.2;
		            //msg.points[0].positions[4] = 0.2;
		            //msg.points[0].positions[5] = 0.2;
		            //msg.points[0].positions[6] = 0.2;
		            
		            msg.points[0].velocities[0] = 0.0;
		            msg.points[0].velocities[1] = 0.0;
		            msg.points[0].velocities[2] = 0.0;
		            msg.points[0].velocities[3] = 0.0;
		            //msg.points[0].velocities[4] = 0.2;
		            //msg.points[0].velocities[5] = 0.2;
		            //msg.points[0].velocities[6] = 0.2;

					// point 2
					msg.points[1].positions[0] = -0.2;
		            msg.points[1].positions[1] = 0.2;
		            msg.points[1].positions[2] = 0.0;
		            msg.points[1].positions[3] = 0.0;
		            //msg.points[1].positions[4] = 0.2;
		            //msg.points[1].positions[5] = 0.2;
		            //msg.points[1].positions[6] = 0.2;
		            
		            msg.points[1].velocities[0] = 0.0;
		            msg.points[1].velocities[1] = 0.0;
		            msg.points[1].velocities[2] = 0.0;
		            msg.points[1].velocities[3] = 0.0;
		            //msg.points[1].velocities[4] = 0.2;
		            //msg.points[1].velocities[5] = 0.2;
		            //msg.points[1].velocities[6] = 0.2;
		            
		            // point 3
					msg.points[2].positions[0] = 0.0;
		            msg.points[2].positions[1] = 0.0;
		            msg.points[2].positions[2] = 0.0;
		            msg.points[2].positions[3] = 0.0;

		            msg.points[2].velocities[0] = 0.0;
		            msg.points[2].velocities[1] = 0.0;
		            msg.points[2].velocities[2] = 0.0;
		            msg.points[2].velocities[3] = 0.0;
                }
                else if (c == '9')
                {
                    msg.points[0].positions[0] = -0.1;
		            msg.points[0].positions[1] = -0.1;
		            msg.points[0].positions[2] = -0.1;
		            msg.points[0].positions[3] = -0.1;
		            //msg.points[0].positions[4] = -0.1;
		            //msg.points[0].positions[5] = -0.1;
		            //msg.points[0].positions[6] = -0.1;
		            
		            msg.points[0].velocities[0] = -0.1;
		            msg.points[0].velocities[1] = -0.1;
		            msg.points[0].velocities[2] = -0.1;
		            msg.points[0].velocities[3] = -0.1;
		            //msg.points[0].velocities[4] = -0.1;
		            //msg.points[0].velocities[5] = -0.1;
		            //msg.points[0].velocities[6] = -0.1;
                }
                else if (c == '8')
                {
                    msg.points[0].positions[0] = -0.2;
		            msg.points[0].positions[1] = -0.2;
		            msg.points[0].positions[2] = -0.2;
		            msg.points[0].positions[3] = -0.2;
		            //msg.points[0].positions[4] = -0.2;
		            //msg.points[0].positions[5] = -0.2;
		            //msg.points[0].positions[6] = -0.2;
		            
		            msg.points[0].velocities[0] = -0.2;
		            msg.points[0].velocities[1] = -0.2;
		            msg.points[0].velocities[2] = -0.2;
		            msg.points[0].velocities[3] = -0.2;
		            //msg.points[0].velocities[4] = -0.2;
		            //msg.points[0].velocities[5] = -0.2;
		            //msg.points[0].velocities[6] = -0.2;
				}
                else
                {
                    ROS_ERROR("invalid target");
                }
                    
                topicPub_JointCommand.publish(msg);
            
                std::cout << std::endl;
                srv_querry = true;
                srv_execute = 0;
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
				ROS_ERROR("Service execution failed. Error message: %s", srv_errorMessage.c_str());
			}
		}
		
	} //while

    return 0;
} //main
