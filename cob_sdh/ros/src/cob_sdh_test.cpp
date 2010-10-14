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
 * ROS package name: cob_sdh
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
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
#include <unistd.h>

// ROS includes
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// ROS message includes
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

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
	actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> ac("command", true);  
        
	// topics to subscribe, callback is called for new messages arriving
    //--
    
    // service servers
    //--
        
    // service clients
    ros::ServiceClient srvClient_Init = n.serviceClient<cob_srvs::Trigger>("init");
    ros::ServiceClient srvClient_Stop = n.serviceClient<cob_srvs::Trigger>("stop");
    ros::ServiceClient srvClient_SetOperationMode = n.serviceClient<cob_srvs::SetOperationMode>("set_operation_mode");
    
    // external code
	bool srv_querry = false;
	int srv_execute = 1;
	std::string srv_errorMessage = "no error";
    
    char c;
         
    // main loop
    while(n.ok())
    {
        // process user inputs
		std::cout << "Choose service to test ([s]top, [i]nit, setOperation[M]ode, send[C]ommand, [e]xit): ";
        
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
				//ROS_INFO("Waiting for action server to start.");
				//ac.waitForServer(); //will wait for infinite time
				
				pr2_controllers_msgs::JointTrajectoryGoal goal;
				
				XmlRpc::XmlRpcValue traj_param;
				trajectory_msgs::JointTrajectory traj;
				traj.points.resize(1);
				
				if (n.hasParam("JointTraj"))
				{
					n.getParam("JointTraj", traj_param);
				}
				else
				{
					ROS_ERROR("Parameter JointTraj not set");
				}
				
				for (int i = 0; i < traj_param.size(); i++)
				{
					std::cout << "traj " << i << " = " << traj_param[i] <<std::endl;
				}
				
				int traj_nr;
				std::cout << traj_param.size() << " trajectories available. Choose trajectory number [0, 1, 2, ...]: ";
                std::cin >> traj_nr;
                std::cout << std::endl;
                
                if (traj_nr < 0 || traj_nr > traj_param.size()-1)
                {
                	ROS_ERROR("traj_nr not in range. traj_nr requested was %d and should be between 0 and %d",traj_nr ,traj_param.size()-1);
                	break;
                }
				
				traj.points[0].positions.resize(traj_param[traj_nr].size());
				for (int i = 0; i<traj_param[traj_nr].size(); i++ )
				{
					traj.points[0].positions[i] = (double)traj_param[traj_nr][i];
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
                ROS_ERROR("invalid input, try again...");
            }
        } //switch
        
		if (!srv_querry)
		{
			ROS_ERROR("Failed to call service");
		}
		else
		{
			ROS_DEBUG("Service call succesfull");
			
			if (srv_execute != 0)
			{
				ROS_ERROR("Service execution failed, errorMessage: %s", srv_errorMessage.c_str());
			}
		}
		
	} //while

    return 0;
} //main
