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
 * ROS stack name:
 * ROS package name:
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author:
 * Supervised by:
 *
 * Date of creation:
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
#include <std_msgs/String.h>

// ROS service includes
#include <std_srvs/Empty.h>

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
        ros::Publisher topicPub_demoPublish;
        
	    // topics to subscribe, callback is called for new messages arriving
        ros::Subscriber topicSub_demoSubscribe;
        
        // service servers
        //--
            
        // service clients
        //--
        
        // global variables
        //--

        // Constructor
        NodeClass()
        {
            topicPub_demoPublish = n.advertise<std_msgs::String>("demoPublish", 1);
            topicSub_demoSubscribe = n.subscribe("demoSubscribe", 1, &NodeClass::topicCallback_demoSubscribe, this);
        }
        
        // Destructor
        ~NodeClass() 
        {
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_demoSubscribe(const std_msgs::String::ConstPtr& msg)
        {
            ROS_INFO("this is topicCallback_demoSubscribe");
        }

        // service callback functions
        // function will be called when a service is querried
        bool srvCallback_demoService(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res )
        {
            ROS_INFO("This is srvCallback_demoService");
            return true;
        }
        
        // other function declarations
        void demoFunction();
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "node_name");
    
    NodeClass nodeClass;
 
    while(nodeClass.n.ok())
    {

        ros::spinOnce();
    }
    
//    ros::spin();

    return 0;
}

//##################################
//#### function implementations ####
void NodeClass::demoFunction()
{
    ROS_INFO("This is the demoFunction");
}
