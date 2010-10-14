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
 * ROS package name: cob_camera_axis
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * Supervised by: Ulrich Reiser, email:ulrich.reiser@ipa.fhg.de
 *
 * Date of creation: April 2010
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	 * Redistributions of source code must retain the above copyright
 *	   notice, this list of conditions and the following disclaimer.
 *	 * Redistributions in binary form must reproduce the above copyright
 *	   notice, this list of conditions and the following disclaimer in the
 *	   documentation and/or other materials provided with the distribution.
 *	 * Neither the name of the Fraunhofer Institute for Manufacturing 
 *	   Engineering and Automation (IPA) nor the names of its
 *	   contributors may be used to endorse or promote products derived from
 *	   this software without specific prior written permission.
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
//#include <cob_srvs/SetOperationMode.h>

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
	ros::init(argc, argv, "camera_axis_test");

	// create a handle for this node, initialize node
	ros::NodeHandle n;

	// topics to publish
	actionlib::SimpleActionClient<cob_actions::JointCommandAction> ac("joint_trajectory_action", true);
	ros::Publisher topic_pub_joint_commands = n.advertise<cob_msgs::JointCommand>("/head_controller/commands", 50);
		
	// topics to subscribe, callback is called for new messages arriving
	//--
	
	// service servers
	//--
		
	// service clients
	ros::ServiceClient srvClient_Init = n.serviceClient<cob_srvs::Trigger>("Init");
	ros::ServiceClient srvClient_Stop = n.serviceClient<cob_srvs::Trigger>("Stop");
	//ros::ServiceClient srvClient_SetOperationMode = n.serviceClient<cob_srvs::SetOperationMode>("SetOperationMode");
	
	// external code
	bool srv_querry = false;
	int srv_execute = 1;
	std::string srv_errorMessage = "no error";
	
	char c;
		 
	// main loop
	while(n.ok())
	{
		// process user inputs
		std::cout << "Choose service to test ([s]top, [i]nit, send[C]ommand, [e]xit, [J]ointCommand): ";
		
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

			case 'C':
			{
				//ROS_INFO("Waiting for action server to start.");
				//ac.waitForServer(); //will wait for infinite time
				
				cob_actions::JointCommandGoal goal;
				
				XmlRpc::XmlRpcValue command_param;
				cob_msgs::JointCommand command;
				
				if (n.hasParam("JointCommand"))
				{
					n.getParam("JointCommand", command_param);
				}
				else
				{
					ROS_ERROR("Parameter JointCommand not set");
				}
				
				for (int i = 0; i < command_param.size(); i++)
				{
					std::cout << "command " << i << " = " << command_param[i] <<std::endl;
				}
				
				int command_nr;
				std::cout << command_param.size() << " commands available. Choose command number [0, 1, 2, ...]: ";
				std::cin >> command_nr;
				std::cout << std::endl;
				
				if (command_nr < 0 || command_nr > command_param.size()-1)
				{
					ROS_ERROR("command_nr not in range. command_nr requested was %d and should be between 0 and %d",command_nr ,command_param.size()-1);
					break;
				}
				
				command.positions.resize(command_param[command_nr].size());
				for (int i = 0; i<command_param[command_nr].size(); i++ )
				{
					command.positions[i] = (double)command_param[command_nr][i];
				}
				
				goal.command = command;
				ac.sendGoal(goal);
				
				std::cout << std::endl;
				srv_querry = true;
				srv_execute = 0;
				break;
			}
			
			case 'J':
			{
				double target_pos;
				double max_speed;
				cob_msgs::JointCommand send_cmd;
				std::cout << "Enter a position value in Radians that should be driven to" << std::endl;
				std::cin >> target_pos;
				std::cout << "Enter a speed value" << std::endl;
				std::cin >> max_speed;
				
				send_cmd.positions.resize(1);
				send_cmd.velocities.resize(1);
				
				send_cmd.positions[0] = target_pos;
				send_cmd.velocities[0] = max_speed;
				
				topic_pub_joint_commands.publish(send_cmd);
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
