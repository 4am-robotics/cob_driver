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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// ROS message includes
//#include <cob_msgs/JointCommand.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <cob_actions/JointTrajectoryAction.h>

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
	ros::init(argc, argv, "sdh_test");

	// create a handle for this node, initialize node
	ros::NodeHandle n;

    // topics to publish
//    ros::Publisher topicPub_JointCommand = n.advertise<cob_msgs::JointCommand>("joint_commands", 1);
    ros::Publisher topicPub_JointCommand = n.advertise<trajectory_msgs::JointTrajectory>("command", 1);
	actionlib::SimpleActionClient<cob_actions::JointTrajectoryAction> ac("JointTrajectory", true); 
        
	// topics to subscribe, callback is called for new messages arriving
    //--
    
    // service servers
    //--
        
    // service clients
    ros::ServiceClient srvClient_Init = n.serviceClient<cob_srvs::Trigger>("Init");
    ros::ServiceClient srvClient_Stop = n.serviceClient<cob_srvs::Trigger>("Stop");
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
        std::cout << "Choose to test ([i]nit, send[C]ommand, [e]xit): ";
        
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
            	ROS_INFO("Waiting for action server to start.");
				// wait for the action server to start
				ac.waitForServer(); //will wait for infinite time
            	
                std::cout << "Choose preset target positions/velocities ([0] = , [1] = , [2] = ): ";
                std::cin >> c;
                
                int DOF = 7;
                
                // send a goal to the action 
				cob_actions::JointTrajectoryGoal goal;
				trajectory_msgs::JointTrajectory traj;
				traj.header.stamp = ros::Time::now();
				
                if (c == '0')
                {
					traj.points.resize(1);
					traj.points[0].positions.resize(DOF);
					traj.points[0].velocities.resize(DOF);
					
					// first point
					// zero position
                }
                else if (c == '1')
                {
					traj.points.resize(1);
					traj.points[0].positions.resize(DOF);
					traj.points[0].velocities.resize(DOF);                                    

					// first point
					traj.points[0].positions[0] = 0.1;
					traj.points[0].positions[1] = 0.1;
					traj.points[0].positions[2] = 0.1;
					traj.points[0].positions[3] = 0.1;
				}
                else if (c == '2')
                {
					traj.points.resize(1);
					traj.points[0].positions.resize(DOF);
					traj.points[0].velocities.resize(DOF);                                    

					// first point
					traj.points[0].positions[0] = 0.2;
					traj.points[0].positions[1] = 0.2;
					traj.points[0].positions[2] = 0.2;
					traj.points[0].positions[3] = 0.2;
                }
                else
                {
                    ROS_ERROR("invalid target");
                }

				goal.trajectory = traj;
				ac.sendGoal(goal);
            
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
				ROS_ERROR("Service execution failed, errorMessage: %s", srv_errorMessage.c_str());
			}
		}
		
	} //while

    return 0;
} //main
