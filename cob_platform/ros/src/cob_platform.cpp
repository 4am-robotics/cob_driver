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
 * ROS package name: cob3_platform
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
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
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// ROS service includes
#include <cob_srvs/Trigger.h>

// external includes
#include <cob_platform/PlatformHardware.h>

//####################
//#### node class ####
class NodeClass
{
    //
    public:
	    // create a handle for this node, initialize node
	    ros::NodeHandle n;
                
        // topics to publish
        ros::Publisher topicPub_Pose2D;
        ros::Publisher topicPub_Odometry;
        tf::TransformBroadcaster odom_broadcaster;

	    // topics to subscribe, callback is called for new messages arriving
        ros::Subscriber topicSub_CmdVel;
        
        // service servers
        ros::ServiceServer srvServer_Init;
        ros::ServiceServer srvServer_Home;
        ros::ServiceServer srvServer_Stop;
        ros::ServiceServer srvServer_Shutdown;
            
        // service clients
        //--
        
        // global variables
        PlatformHardware* pltf;
        bool isInitialized;
        double cmdVelX, cmdVelY, cmdVelTh;
        double x, y, th;
        double dxMM, dyMM, dth, dvth;
        double vxMMS, vyMMS, vth, vvth;
        ros::Time current_time, last_time;

        // Constructor
        NodeClass()
        {
            // initialize global variables
            isInitialized = false;
            last_time = ros::Time::now();
	        current_time = ros::Time::now();
            cmdVelX = cmdVelY = cmdVelTh = 0;
            x = y = th = 0;
            dxMM = dyMM = dth = dvth = 0;
            vxMMS = vyMMS = vth = vvth = 0;

        	// implementation of topics to publish
            topicPub_Odometry = n.advertise<nav_msgs::Odometry>("odometry", 50);

            // implementation of topics to subscribe
            topicSub_CmdVel = n.subscribe("cmd_vel", 1, &NodeClass::topicCallback_CmdVel, this);
            
            // implementation of service servers
            srvServer_Init = n.advertiseService("Init", &NodeClass::srvCallback_Init, this);
            srvServer_Stop = n.advertiseService("Stop", &NodeClass::srvCallback_Stop, this);
            srvServer_Shutdown = n.advertiseService("Shutdown", &NodeClass::srvCallback_Shutdown, this);
        }
        
        // Destructor
        ~NodeClass() 
        {
            pltf->setVelPltf(0, 0, 0, 0);
            pltf->shutdownPltf();
            isInitialized = false;
            delete pltf;
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_CmdVel(const geometry_msgs::Twist::ConstPtr& msg)
        {
            ROS_DEBUG("received new velocity command [cmdVelX=%3.5f,cmdVelY=%3.5f,cmdVelTh=%3.5f]", 
                     msg->linear.x, msg->linear.y, msg->angular.z);

            cmdVelX = msg->linear.x;
            cmdVelY = msg->linear.y;
            cmdVelTh = msg->angular.z;
        }

        // service callback functions
        // function will be called when a service is querried
        bool srvCallback_Init(cob_srvs::Trigger::Request &req,
                              cob_srvs::Trigger::Response &res )
        {
            if(isInitialized == false)
            {
                ROS_INFO("...initializing platform...");
                pltf = new PlatformHardware();
                pltf->initPltf();
                isInitialized = true;
                res.success = 0; // 0 = true, else = false
            }
            else
            {
                ROS_ERROR("...platform already initialized...");
                res.success = 1;
                res.errorMessage.data = "platform already initialized";
            }            
            return true;
        }
        
        bool srvCallback_Stop(cob_srvs::Trigger::Request &req,
                              cob_srvs::Trigger::Response &res )
        {
            if(isInitialized == true)
            {
                ROS_INFO("...stopping platform...");
                cmdVelX = 0;
                cmdVelY = 0;
                cmdVelTh = 0;
                res.success = 0; // 0 = true, else = false
            }
            else
            {
                ROS_ERROR("...platform not initialized...");
                res.success = 1;
                res.errorMessage.data = "platform not initialized";
            }
            return true;
        }

        bool srvCallback_Shutdown(cob_srvs::Trigger::Request &req,
                                  cob_srvs::Trigger::Response &res )
        {
            if(isInitialized == true)
            {
                ROS_INFO("...shutting down platform...");
                pltf->shutdownPltf();
                isInitialized = false;
                res.success = 0; // 0 = true, else = false
            }            
            else
            {
                ROS_ERROR("...platform not initialized...");
                res.success = 1;
                res.errorMessage.data = "platform not initialized";
            }
            return true;
        }
        
        // other function declarations
        void updateCmdVel()
        {
            // send vel if platform is initialized
            if(isInitialized == true)
            {
                ROS_DEBUG("update cmdVel");
                pltf->setVelPltf(cmdVelX*1000, cmdVelY*1000, cmdVelTh, 0);  // convert from m/s to mm/s
            }
        }

        void updateOdometry()
        {
            if(isInitialized == true)
            {
				double u_x,u_y,dt = 0.0;
	            current_time = ros::Time::now();
				dt = current_time.toSec() - last_time.toSec();
				last_time = current_time;

                // get odometry from platform
                pltf->getDeltaPosePltf(dxMM, dyMM, dth, dvth,
        					           vxMMS, vyMMS, vth, vvth);

				// calculation from cpc
/*				if (fabs(vth) < 0.05)
				{
					u_x = vxMMS*dt/1000.0;
					u_y = vyMMS*dt/1000.0;
				}
				else
				{
					u_x = vxMMS*sin(vth*dt)/vth/1000.0 + vyMMS/vth/1000.0*(cos(vth*dt)-1.0);
					u_y = -vxMMS/vth/1000.0*(cos(vth*dt)-1.0) + vyMMS*sin(vth*dt)/vth/1000.0;
					if (fabs(u_x)>1 || fabs(u_y)>1)
					{
						ROS_INFO("u_x = %f, u_y=%f, vth=%f, vx=%f, vy=%f",u_x,u_y,vth,vxMMS,vyMMS);
					}
				}

                // add delta values to old values
                x += u_x*cos(th)-u_y*sin(th);
                y += u_x*sin(th)+u_y*cos(th);
                th += vth*dt;
*/

				// calculation from ROS odom publisher tutorial http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Odom
			    //compute odometry in a typical way given the velocities of the robot
				//double dt = (current_time - last_time).toSec();
				double delta_x = (vxMMS/1000.0 * cos(th) - vyMMS/1000.0 * sin(th)) * dt;
				double delta_y = (vxMMS/1000.0 * sin(th) + vyMMS/1000.0 * cos(th)) * dt;
				double delta_th = vth * dt;

				x += delta_x;
				y += delta_y;
				th += delta_th;




			    //since all odometry is 6DOF we'll need a quaternion created from yaw
				geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

				//first, we'll publish the transform over tf
				geometry_msgs::TransformStamped odom_trans;
				odom_trans.header.stamp = current_time;
				odom_trans.header.frame_id = "/odom";
				odom_trans.child_frame_id = "/base_footprint";

				odom_trans.transform.translation.x = x;
				odom_trans.transform.translation.y = y;
				odom_trans.transform.translation.z = 0.0;
				odom_trans.transform.rotation = odom_quat;

				//send the transform
				odom_broadcaster.sendTransform(odom_trans);



                //next, we'll publish the odometry message over ROS
                nav_msgs::Odometry odom;
                odom.header.stamp = current_time;
                odom.header.frame_id = "/odom";
                odom.child_frame_id = "/base_footprint";

                //set the position
                odom.pose.pose.position.x = x;
                odom.pose.pose.position.y = y;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = odom_quat;

                //set the velocity
                odom.twist.twist.linear.x = vxMMS / 1000.0; // convert from mm/s to m/s
                odom.twist.twist.linear.y = vyMMS / 1000.0; // convert from mm/s to m/s
                odom.twist.twist.angular.z = vth;

                //publish the message
                //topicPub_Odometry.publish(odom);

				last_time = current_time;
            }
        }
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "cob3_platform");
    
    NodeClass nodeClass;

    // main loop
 	ros::Rate loop_rate(10); // Hz 
    while(nodeClass.n.ok())
    {
        nodeClass.current_time = ros::Time::now();

        // publish Odometry
        nodeClass.updateOdometry();

        // update velocity of platform
        nodeClass.updateCmdVel();

        ros::spinOnce();
        loop_rate.sleep();
    }
    
//    ros::spin();

    return 0;
}
