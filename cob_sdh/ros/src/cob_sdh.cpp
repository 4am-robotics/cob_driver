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

// geändert

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <cob_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>

// ROS service includes
#include <cob_srvs/Trigger.h>

// external includes
#include <cob_sdh/sdh.h>

//####################
//#### node class ####
class NodeClass
{
    //
    public:
	    // create a handle for this node, initialize node
	    ros::NodeHandle n;
                
        // declaration of topics to publish
        ros::Publisher topicPub_JointState;
        ros::Publisher topicPub_ActuatorState;
        
	    // declaration of topics to subscribe, callback is called for new messages arriving
        ros::Subscriber topicSub_JointCommand;
        
        // service servers
        ros::ServiceServer srvServer_Init;
            
        // service clients
        //--
        
        // global variables
        cSDH *sdh;
        bool isInitialized;
        int DOF;

        // Constructor
        NodeClass()
        {
	        // initialize global variables
            isInitialized = false;
            DOF = 7; // DOFs of sdh

            // implementation of topics to publish
            topicPub_JointState = n.advertise<sensor_msgs::JointState>("joint_states", 1);
            
            // implementation of topics to subscribe
            topicSub_JointCommand = n.subscribe("joint_commands", 1, &NodeClass::topicCallback_JointCommand, this);
            
            // implementation of service servers
            srvServer_Init = n.advertiseService("Init", &NodeClass::srvCallback_Init, this);
        }
        
        // Destructor
        ~NodeClass() 
        {
        	sdh->Close();
        	delete sdh;
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_JointCommand(const cob_msgs::JointCommand::ConstPtr& msg)
        {
            ROS_INFO("Received new JointCommand");
            
            if(isInitialized == true)
            {
				//TODO: send msg data to hardware
			  	std::vector<int> axes;
				for(int i=0; i<DOF; i++)
				{
					axes.push_back(i);
				}
	
				try
				{
					sdh->SetAxisTargetAngle( axes, msg->positions );
				}
				catch (cSDHLibraryException* e)
				{
					ROS_ERROR("An exception was caught: %s", e->what());
					delete e;
				}
		
				try
				{
					sdh->MoveHand(true);
				}
				catch (cSDHLibraryException* e)
				{
					ROS_ERROR("An exception was caught: %s", e->what());
					delete e;
			   	}
			}
			else
			{
				ROS_ERROR("sdh not initialized");
			}
        }

        // service callback functions
        // function will be called when a service is querried
        bool srvCallback_Init(cob_srvs::Trigger::Request &req,
                              cob_srvs::Trigger::Response &res )
        {
        	ROS_INFO("Initializing sdh");
        	
        	//TODO: read from parameter
          	int _net=0;
			unsigned long _baudrate=1000000;
			double _timeout=-1.0;
			unsigned int _id_read=43;
			unsigned int _id_write=42;
			
			if(isInitialized == false)
			{
				ROS_INFO("...initializing sdh...");
				sdh = new cSDH();
				try
				{
					sdh->OpenCAN_ESD( _net, _baudrate, _timeout, _id_read, _id_write );
				}
				catch (cSDHLibraryException* e)
				{
					ROS_ERROR("Initializing aborted. An exception was caught: %s", e->what());
					isInitialized = false;
					res.success = 1; // 0 = true, else = false
					delete e;
					return true;
				}
				ROS_INFO("Initializing succesfull");
				isInitialized = true;
				res.success = 0; // 0 = true, else = false
			}
			else
			{
				ROS_ERROR("...sdh already initialized...");
                res.success = 1;
                res.errorMessage.data = "sdh already initialized";
			}
            return true;
        }
        
        // other function declarations
        void updateJointState()
        {
		    ROS_INFO("updateJointState");
		    std::vector<double> actualAngles;
		    
		    if(isInitialized == true)
			{
		        //get actual joint positions 
		        std::vector<int> axes;

				for(int i=0; i<DOF; i++)
				{
					axes.push_back(i);
				}

		        try
		        {
					actualAngles = sdh->GetAxisActualAngle( axes );
				}
				catch (cSDHLibraryException* e)
				{
					ROS_ERROR("An exception was caught: %s", e->what());
					delete e;
			   	}
			}
			else
			{
				actualAngles.resize(DOF);
				for(int i=0; i<DOF; i++)
				{
					actualAngles[i] = 0.0;
				}
			}

			// fill message
			// NOTE: hardware has 8 DOFs, but two cuppled ones; joints palm_finger11 and palm_finger21 are actuated synchronously
			sensor_msgs::JointState msg;
			msg.header.stamp = ros::Time::now();
			msg.name.resize(DOF+1);
			msg.position.resize(DOF+1);

			// set joint names and map them to angles TODO: don't know if assignment is correct
			msg.name[0] = "joint_thumb1_thumb2";
			msg.position[0] = actualAngles[0];
			msg.name[1] = "joint_thumb2_thumb3";
			msg.position[0] = actualAngles[0];
			msg.name[2] = "joint_palm_finger11";
			msg.position[0] = actualAngles[0];
			msg.name[3] = "joint_finger11_finger12";
			msg.position[0] = actualAngles[0];
			msg.name[4] = "joint_finger12_finger13";
			msg.position[0] = actualAngles[0];
			msg.name[5] = "joint_palm_finger21";
			msg.position[0] = actualAngles[0];
			msg.name[6] = "joint_finger21_finger22";
			msg.position[0] = actualAngles[0];
			msg.name[7] = "joint_finger22_finger23";
			msg.position[0] = actualAngles[0];

	        //publish the message
	        topicPub_JointState.publish(msg);

	        ROS_INFO("published JointState");
        }
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "sdh");
	ROS_INFO("...sdh node running...");
    
    NodeClass nodeClass;
 
 	ros::Rate loop_rate(5); // Hz
    while(nodeClass.n.ok())
    {
        // publish JointState
        nodeClass.updateJointState();
    
        // sleep and waiting for messages, callbacks    
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
