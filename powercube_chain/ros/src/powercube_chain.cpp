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
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <cob_msgs/JointCommand.h>

// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>

// external includes
#include <powercube_chain/PowerCubeCtrl.h>

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
        
	    // declaration of topics to subscribe, callback is called for new messages arriving
        ros::Subscriber topicSub_JointCommand;
        
        // declaration of service servers
        ros::ServiceServer srvServer_Init;
        ros::ServiceServer srvServer_Stop;
        ros::ServiceServer srvServer_SetOperationMode;
            
        // declaration of service clients
        //--
        
        // global variables
        PowerCubeCtrl* PCube;

        // Constructor
        NodeClass()
        {
        	PCube = new PowerCubeCtrl();
        
            // implementation of topics to publish
            topicPub_JointState = n.advertise<sensor_msgs::JointState>("joint_states", 1);
            
            // implementation of topics to subscribe
            topicSub_JointCommand = n.subscribe("joint_commands", 1, &NodeClass::topicCallback_JointCommand, this);
            
            // implementation of service servers
            srvServer_Init = n.advertiseService("Init", &NodeClass::srvCallback_Init, this);
            srvServer_Stop = n.advertiseService("Stop", &NodeClass::srvCallback_Stop, this);
            srvServer_SetOperationMode = n.advertiseService("SetOperationMode", &NodeClass::srvCallback_SetOperationMode, this);
            
            // implementation of service clients
            //--
        }
        
        // Destructor
        ~NodeClass() 
        {
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_JointCommand(const cob_msgs::JointCommand::ConstPtr& msg)
        {
            std::string operationMode;
            n.getParam("OperationMode", operationMode);
            if (operationMode == "position")
            {
                ROS_INFO("moving powercubes in position mode");
                //TODO PowerCubeCtrl
                PCube->MoveJointSpaceSync(msg->positions);
            }
            else if (operationMode == "velocity")
            {
            	ROS_INFO("moving powercubes in velocity mode");
                //TODO PowerCubeCtrl
                PCube->MoveVel(msg->velocities);  
            }
            else
            {
                ROS_ERROR("powercubes neither in position nor in velocity mode. OperationMode = [%s]", operationMode.c_str());
            }
        }

        // service callback functions
        // function will be called when a service is querried
        bool srvCallback_Init(cob_srvs::Trigger::Request &req,
                              cob_srvs::Trigger::Response &res )
        {
        	ROS_INFO("Initializing powercubes");
          	
          	// init powercubes 
          	//TODO: make iniFilepath as an argument
          	//TODO: read iniFile into ros prarameters
            if (PCube->Init("include/iniFile.txt")) 
            {
            	ROS_INFO("Initializing succesfull");
            	res.success = 0; // 0 = true, else = false
            }
            else
            {
            	ROS_ERROR("Initializing powercubes not succesfull. error: %s", PCube->getErrorMessage().c_str());
            	res.success = 1; // 0 = true, else = false
            	res.errorMessage.data = PCube->getErrorMessage();
            	return true;
            }
            
            // homing powercubes
            if (PCube->doHoming())
            {
            	ROS_INFO("Homing powercubes succesfull");
            	res.success = 0; // 0 = true, else = false
            }
            else
            {
            	ROS_ERROR("Homing powercubes not succesfull. error: %s", PCube->getErrorMessage().c_str());
            	res.success = 1; // 0 = true, else = false
            	res.errorMessage.data = PCube->getErrorMessage();
            	return true;
            }
            return true;
        }

        bool srvCallback_Stop(cob_srvs::Trigger::Request &req,
                              cob_srvs::Trigger::Response &res )
        {
       	    ROS_INFO("Stopping powercubes");
        
            // stopping all arm movements
            if (PCube->Stop())
            {
            	ROS_INFO("Stopping powercubes succesfull");
            	res.success = 0; // 0 = true, else = false
            }
            else
            {
            	ROS_ERROR("Stopping powercubes not succesfull. error: %s", PCube->getErrorMessage().c_str());
            	res.success = 1; // 0 = true, else = false
            	res.errorMessage.data = PCube->getErrorMessage();
            }
            return true;
        }

        bool srvCallback_SetOperationMode(cob_srvs::SetOperationMode::Request &req,
                                          cob_srvs::SetOperationMode::Response &res )
        {
        	ROS_INFO("Set operation mode to [%s]", req.operationMode.data.c_str());
            n.setParam("OperationMode", req.operationMode.data.c_str());
            res.success = 0; // 0 = true, else = false
            return true;
        }
                
        // other function declarations
        void publishJointState()
        {
            // create message
            int DOF = 7;
            std::vector<double> ActualPos;
            std::vector<double> ActualVel;
            ActualPos.resize(DOF);
            ActualVel.resize(DOF);
            //TODO
            //PCube->getConfig(ActualPos);
            //get velocities
            sensor_msgs::JointState msg;
            msg.header.stamp = ros::Time::now();
            msg.name.resize(DOF);
            msg.position.resize(DOF);
            msg.velocity.resize(DOF);
            
            // TODO: read joint names from parameter file
            msg.name[0] = "joint_arm0_arm1";
            msg.name[1] = "joint_arm1_arm2";
            msg.name[2] = "joint_arm2_arm3";
            msg.name[3] = "joint_arm3_arm4";
            msg.name[4] = "joint_arm4_arm5";
            msg.name[5] = "joint_arm5_arm6";
            msg.name[6] = "joint_arm6_arm7";
            
            for (int i = 0; i<DOF; i++ )
            {
                msg.position[i] = ActualPos[i];
                msg.velocity[i] = ActualVel[i];
            }
                
            // publish message
            ROS_INFO("published ActualPos [%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f]", 
                     msg.position[0], msg.position[1], msg.position[2], msg.position[3], 
                     msg.position[4], msg.position[5], msg.position[6]);
            topicPub_JointState.publish(msg);
        }

}; //NodeClass

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "powercube_chain");
    
    // create nodeClass
    NodeClass nodeClass;
 
    // main loop
 	ros::Rate loop_rate(5); // Hz
    while(nodeClass.n.ok())
    {
        // publish JointState
        nodeClass.publishJointState();

        // read parameter
        std::string operationMode;
        nodeClass.n.getParam("OperationMode", operationMode);
        ROS_INFO("running with OperationMode [%s]", operationMode.c_str());

        // sleep and waiting for messages, callbacks 
        ros::spinOnce();
        loop_rate.sleep();
    }
    
//    ros::spin();

    return 0;
}
