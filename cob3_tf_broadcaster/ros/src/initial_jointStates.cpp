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
 * ROS package name: cob3_tf_broadcaster
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
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/JointState.h>

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
		ros::Publisher topicPub_JointState;
        
	    // topics to subscribe, callback is called for new messages arriving
        //--
        
        // service servers
        //--
            
        // service clients
        //--
        
        // global variables
		//--

        // Constructor
        NodeClass()
        {
        	topicPub_JointState = n.advertise<sensor_msgs::JointState>("joint_states", 50);
        }
        
        // Destructor
        ~NodeClass() 
        {
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
		//--

        // service callback functions
        // function will be called when a service is querried
		//--
        
        // other function declarations
        void init_torso(double val)
        {
		    sensor_msgs::JointState jointState;
			jointState.header.stamp = ros::Time::now();
		    
		    int DOF = 5;
			
			jointState.set_name_size(DOF);
		    jointState.set_position_size(DOF);
		    jointState.set_velocity_size(DOF);
		    
		    jointState.name[0] = "joint_platform_neckZ";
		    jointState.name[1] = "joint_neckZ_neck";
		    jointState.name[2] = "joint_neck_headZ";
		    jointState.name[2] = "joint_headZ_head";
		    jointState.name[3] = "joint_headZ_head_cover";
		    jointState.name[4] = "joint_head_eyes";
		    
            for (int i = 0; i<DOF; i++ )
            {
                jointState.position[i] = val;
                jointState.velocity[i] = 0.0;
            }
		           
		    // publish message
		    topicPub_JointState.publish(jointState);
		}
		
        void init_arm(double val)
        {
		    		    sensor_msgs::JointState jointState;
			jointState.header.stamp = ros::Time::now();
		    
		    int DOF = 7;
			
			jointState.set_name_size(DOF);
		    jointState.set_position_size(DOF);
		    jointState.set_velocity_size(DOF);
		    
		    jointState.name[0] = "joint_arm0_arm1";
		    jointState.name[1] = "joint_arm1_arm2";
		    jointState.name[2] = "joint_arm2_arm3";
		    jointState.name[3] = "joint_arm3_arm4";
		    jointState.name[4] = "joint_arm4_arm5";
		    jointState.name[5] = "joint_arm5_arm6";
		    jointState.name[6] = "joint_arm6_arm7";
		    
            for (int i = 0; i<DOF; i++ )
            {
                jointState.position[i] = val;
                jointState.velocity[i] = 0.0;
            }

		    // publish message
		    topicPub_JointState.publish(jointState);
		}
		
        void init_sdh(double val)
        {
		    sensor_msgs::JointState jointState;
			jointState.header.stamp = ros::Time::now();
		    
		    int DOF = 9;
			
			jointState.set_name_size(DOF);
		    jointState.set_position_size(DOF);
		    jointState.set_velocity_size(DOF);
		    
		    jointState.name[0] = "joint_palm_finger11";
		    jointState.name[1] = "joint_finger11_finger12";
		    jointState.name[2] = "joint_finger12_finger13";
		    jointState.name[3] = "joint_palm_finger21";
		    jointState.name[4] = "joint_finger21_finger22";
		    jointState.name[5] = "joint_finger22_finger23";
		    jointState.name[6] = "joint_palm_thumb1";
		    jointState.name[7] = "joint_thmub1_thumb2";
		    jointState.name[8] = "joint_thmub2_thumb3";
		    
            for (int i = 0; i<DOF; i++ )
            {
                jointState.position[i] = val;
                jointState.velocity[i] = 0.0;
            }
		           
		    // publish message
		    topicPub_JointState.publish(jointState);
		}
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "initial_jointStates");
    
    NodeClass nodeClass;

    // main loop
 	ros::Rate loop_rate(10); // Hz
	ROS_INFO("publishing initial JointStates");
	int count = 0; // HACK, joint states have to be published multiple times to take effect for robot_state_publisher
    while(count < 100)
    {
    	ROS_INFO("%d --> %f",count,0.1-count/1000.0);
    
		// initial JointStates for torso
		nodeClass.init_torso(0.1-count/1000.0);

		// initial JointStates for arm
		nodeClass.init_arm(0.1-count/1000.0);
		
		// initial JointStates for sdh
		nodeClass.init_sdh(0.1-count/1000.0);

		count++;
        ros::spinOnce();
        loop_rate.sleep();
    }
	ROS_INFO("initial JointStates published");
    return 0;
}
