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
 * ROS package name: cob_relayboard
 * Description: Reads and sends data over Serial Interface to the Serial Relayboard. Main Tasks: Reading of EmergencyStop and LaserScannerStop
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Philipp Koehler
 * Supervised by: Christian Connette
 *
 * Date of creation: March 2010
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
#include <cob_relayboard/SerRelayBoard.h>

// ROS message includes
#include <std_msgs/Bool.h>

// ROS service includes
//--

// external includes
//--

//####################
//#### node class ####
class NodeClass
{
	//
	public:
		// create a handle for this node, initialize node
		ros::NodeHandle n;
                
		// topics to publish
		ros::Publisher topicPub_isEmergencyStop;
		ros::Publisher topicPub_isScannerStop;
        
		// topics to subscribe, callback is called for new messages arriving
		//ros::Subscriber topicSub_demoSubscribe;

		// Constructor
		NodeClass()
		{
			topicPub_isEmergencyStop = n.advertise<std_msgs::Bool>("isEmergencyStop", 1);
			topicPub_isScannerStop = n.advertise<std_msgs::Bool>("isScannerStop", 1);
		}
        
		// Destructor
		~NodeClass() 
		{
			delete m_SerRelayBoard;
		}
    
		void sendEmergencyStopStates();
		int init();

	private:        
		std::string sIniDirectory;
		SerRelayBoard * m_SerRelayBoard;

		int requestBoardStatus();
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "cob_relayboard_node");
    
	NodeClass node;
	if(node.init() != 0) return 1;
 
	ros::Rate r(20); //Cycle-Rate: Frequency of publishing EMStopStates
	while(node.n.ok())
	{        
		node.sendEmergencyStopStates();

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}

//##################################
//#### function implementations ####

int NodeClass::init() {
	n.param<std::string>("cob_relayboard_node/IniDirectory", sIniDirectory, "Platform/IniFiles/");
    
	m_SerRelayBoard = new SerRelayBoard(sIniDirectory);

	m_SerRelayBoard->initPltf();

	return 0;
}

int NodeClass::requestBoardStatus() {
	// Request Status of RelayBoard 
	m_SerRelayBoard->setWheelVel(0,0.0,0);

	m_SerRelayBoard->evalRxBuffer();

	return 0;
}

void NodeClass::sendEmergencyStopStates()
{
	requestBoardStatus();
	std_msgs::Bool msg;

	msg.data = m_SerRelayBoard->isEMStop();
	topicPub_isEmergencyStop.publish(msg);
	msg.data = m_SerRelayBoard->isScannerStop();
	topicPub_isScannerStop.publish(msg);
}

