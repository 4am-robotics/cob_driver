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
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
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
#include <cob_msgs/EmergencyStopState.h>

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
        
		// topics to subscribe, callback is called for new messages arriving
		// --

		// Constructor
		NodeClass()
		{
			topicPub_isEmergencyStop = n.advertise<cob_msgs::EmergencyStopState>("/emergency_stop_state", 1);

			// Make sure member variables have a defined state at the beginning
			EM_stop_status_ = ST_EM_FREE;
			duration_for_EM_free_ = ros::Duration(1);
		}
        
		// Destructor
		~NodeClass() 
		{
			delete m_SerRelayBoard;
		}
    
		void sendEmergencyStopStates();
		int init();

	private:        
		std::string sComPort;
		SerRelayBoard * m_SerRelayBoard;

		int EM_stop_status_;
		ros::Duration duration_for_EM_free_;
		ros::Time time_of_EM_confirmed_;

		// possible states of emergency stop
		enum
		{
			ST_EM_FREE = 0,
			ST_EM_ACTIVE = 1,
			ST_EM_CONFIRMED = 2
		};

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

int NodeClass::init() 
{
	if (n.hasParam("ComPort"))
	{
		n.getParam("ComPort", sComPort);
		ROS_INFO("Loaded ComPort parameter from parameter server: %s",sComPort.c_str());
	}
	else
	{
		sComPort ="/dev/ttyUSB0";
		ROS_WARN("ComPort Parameter not available, using default Port: %s",sComPort.c_str());
	}
    
	m_SerRelayBoard = new SerRelayBoard(sComPort);
	ROS_INFO("Opened Relayboard at ComPort = %s", sComPort.c_str());

	m_SerRelayBoard->init();

	// Init member variable for EM State
	EM_stop_status_ = ST_EM_FREE;
	duration_for_EM_free_ = ros::Duration(1);

	return 0;
}

int NodeClass::requestBoardStatus() {
	int ret;	
	
	// Request Status of RelayBoard 
	ret = m_SerRelayBoard->sendRequest();
	if(ret != SerRelayBoard::NO_ERROR) {
		ROS_ERROR("Error in sending message to Relayboard over SerialIO, lost bytes during writing");
	}

	ret = m_SerRelayBoard->evalRxBuffer();
	if(ret==SerRelayBoard::NOT_INITIALIZED) {
		ROS_ERROR("Failed to read relayboard data over Serial, the device is not initialized");
	} else if(ret==SerRelayBoard::NO_MESSAGES) {
		ROS_ERROR("For a long time, no messages from RelayBoard have been received, check com port!");
	} else if(ret==SerRelayBoard::TOO_LESS_BYTES_IN_QUEUE) {
		//ROS_ERROR("Relayboard: Too less bytes in queue");
	} else if(ret==SerRelayBoard::CHECKSUM_ERROR) {
		ROS_ERROR("A checksum error occurred while reading from relayboard data");
	}

	return 0;
}

void NodeClass::sendEmergencyStopStates()
{
	requestBoardStatus();
	bool EM_signal;
	ros::Duration duration_since_EM_confirmed;
	cob_msgs::EmergencyStopState EM_msg;

	// assign input (laser, button) specific EM state
	EM_msg.emergency_button_stop = m_SerRelayBoard->isEMStop();
	EM_msg.scanner_stop = m_SerRelayBoard->isScannerStop();

	// determine current EMStopState
	EM_signal = (EM_msg.emergency_button_stop || EM_msg.scanner_stop);

	switch (EM_stop_status_)
	{
		case ST_EM_FREE:
		{
			if (EM_signal == true)
			{
				ROS_INFO("Emergency stop was issued");
				EM_stop_status_ = EM_msg.EMSTOP;
			}
			break;
		}
		case ST_EM_ACTIVE:
		{
			if (EM_signal == false)
			{
				ROS_INFO("Emergency stop was confirmed");
				EM_stop_status_ = EM_msg.EMCONFIRMED;
				time_of_EM_confirmed_ = ros::Time::now();
			}
			break;
		}
		case ST_EM_CONFIRMED:
		{
			if (EM_signal == true)
			{
				ROS_INFO("Emergency stop was issued");
				EM_stop_status_ = EM_msg.EMSTOP;
			}
			else
			{
				duration_since_EM_confirmed = ros::Time::now() - time_of_EM_confirmed_;
				if( duration_since_EM_confirmed.toSec() > duration_for_EM_free_.toSec() )
				{
					ROS_INFO("Emergency stop released");
					EM_stop_status_ = EM_msg.EMFREE;
				}
			}
			break;
		}
	};
	EM_msg.emergency_state = EM_stop_status_;
	topicPub_isEmergencyStop.publish(EM_msg);
}

