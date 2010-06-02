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
 * ROS package name: cob_tf_broadcaster
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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

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
		tf::TransformBroadcaster br;
        
	    // topics to subscribe, callback is called for new messages arriving
        //--
        
        // service servers
        //--
            
        // service clients
        //--
        
        // global variables
		tf::StampedTransform transform;
		tf::TransformListener transformListener;

        // Constructor
        NodeClass()
        {
            //--
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
		void updateTf()
        {
            ROS_DEBUG("update static tf cyclically");
			
			// publish tf for base_footprint --> base_link
			transform.setOrigin(tf::Vector3(0.0, 0.0, 0.2));
			transform.setRotation(tf::Quaternion(0, 0, 0));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "base_link"));

			// publish tf for base_link --> base_laser_front
			transform.setOrigin(tf::Vector3(0.325, 0.0, 0.1));
			transform.setRotation(tf::Quaternion(0, 0, 0));
//			transform.setRotation(tf::createQuaternionFromRPY(3.1415926, 0, 0));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "base_laser_front_link"));

			// publish tf for base_link --> base_laser_rear
			transform.setOrigin(tf::Vector3(-0.325, 0.0, 0.1));
			transform.setRotation(tf::createQuaternionFromRPY(0, 0, 3.1415926));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "base_laser_rear_link"));

			// publish tf for base_link --> base_hokuyo
			transform.setOrigin(tf::Vector3(-0.252, 0.0, 0.3));
//			transform.setRotation(tf::Quaternion(0, 0, 0));
			transform.setRotation(tf::createQuaternionFromRPY(0, 0, 3.1415926));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "base_laser_top_link"));

			// publish tf for base_link --> head TODO not static
			transform.setOrigin(tf::Vector3(0.0, 0.0, 0.8));
			transform.setRotation(tf::Quaternion(0, 0, 0));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "head"));

			// publish tf for head --> cameraAxis TODO not static
			transform.setOrigin(tf::Vector3(0.0, 0.0, 0.2));
			transform.setRotation(tf::Quaternion(0, 0, 0));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "head", "cameraAxis"));

			// publish tf for cameraAxis --> colorCam_left
			transform.setOrigin(tf::Vector3(0.0, 0.1, 0.0));
			transform.setRotation(tf::Quaternion(0, 0, 0));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "cameraAxis", "colorCam_left"));

			// publish tf for cameraAxis --> colorCam_right
			transform.setOrigin(tf::Vector3(0.0, -0.1, 0.0));
			transform.setRotation(tf::Quaternion(0, 0, 0));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "cameraAxis", "colorCam_right"));

			// publish ...

        }

};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "cob_tf_broadcaster");
    
    NodeClass nodeClass;

    // main loop
 	ros::Rate loop_rate(10); // Hz
	ROS_INFO("publishing tf with 10 Hz");
    while(nodeClass.n.ok())
    {
        // update static tf
        nodeClass.updateTf();

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
