/*
 * cpc_publisher_main.cpp
 *
 *  Created on: 25.08.2010
 *      Author: goa
 */

// ROS includes
#include <ros/ros.h>
#include <cob_point_cloud_publisher/cob_colored_point_cloud_publisher.h>

// ROS service includes
//--

// external includes
//--
#define NODE 1


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, specify name of node
    ros::init(argc, argv, "cob_colored_point_cloud_publisher");

    PcPublisher pcPublisher;
    pcPublisher.initNode();

    ros::spin();

    return 0;
}
