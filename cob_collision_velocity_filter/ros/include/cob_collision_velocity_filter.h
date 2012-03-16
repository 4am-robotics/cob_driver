/****************************************************************
 *
 * Copyright (c) 2012
 *
 * Fraunhofer Institute for Manufacturing Engineering  
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_navigation
 * ROS package name: cob_collision_velocity_filter
 *  							
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  		
 * Author: Matthias Gruhler, email:Matthias.Gruhler@ipa.fhg.de
 *
 * Date of creation: February 2012
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *  	 notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *  	 notice, this list of conditions and the following disclaimer in the
 *  	 documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Fraunhofer Institute for Manufacturing 
 *  	 Engineering and Automation (IPA) nor the names of its
 *  	 contributors may be used to endorse or promote products derived from
 *  	 this software without specific prior written permission.
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
#ifndef COB_COLLISION_VELOCITY_FILTER_H
#define COB_COLLISION_VELOCITY_FILTER_H

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
#include <XmlRpc.h>

#include <pthread.h>

// ROS message includes
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/GridCells.h>

#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

// ROS service includes
#include "cob_footprint_observer/GetFootprint.h"

///
/// @class CollisionVelocityFilter
/// @brief checks for obstacles in driving direction and stops the robot
/// 
///
class CollisionVelocityFilter
{
  public:

    ///
    /// @brief  Constructor
    ///
    CollisionVelocityFilter();

    
    ///
    /// @brief  Destructor
    ///
    ~CollisionVelocityFilter();

    ///
    /// @brief  reads twist command from teleop device (joystick, teleop_keyboard, ...) and calls functions 
    ///         for collision check (obstacleHandler) and driving of the robot (performControllerStep)
    /// @param  twist - velocity command sent as twist message (twist.linear.x/y/z, twist.angular.x/y/z) 
    ///
    void joystickVelocityCB(const geometry_msgs::Twist::ConstPtr &twist);

    ///
    /// @brief  reads obstacles from costmap
    /// @param  obstacles - 2D occupancy grid in rolling window mode!
    ///
    void obstaclesCB(const nav_msgs::GridCells::ConstPtr &obstacles);


    ///
    /// @brief  Timer callback, calls GetFootprint Service and adjusts footprint
    ///
    void getFootprintServiceCB(const ros::TimerEvent&);
    
    /// create a handle for this node, initialize node
    ros::NodeHandle nh_;

    /// Timer for periodically calling GetFootprint Service
    ros::Timer get_footprint_timer_;
    
    /// declaration of publisher 
    ros::Publisher topic_pub_command_;
    ros::Publisher topic_pub_relevant_obstacles_;

    /// declaration of subscriber
    ros::Subscriber joystick_velocity_sub_, obstacles_sub_;

    /// declaration of service client
    ros::ServiceClient srv_client_get_footprint_;

  private:
    /* core functions */
    
    ///
    /// @brief  checks distance to obstacles in driving direction and slows down/stops 
    ///         robot and publishes command velocity to robot
    ///
    void performControllerStep();
    
    ///
    /// @brief  checks for obstacles in driving direction of the robot (rotation included) 
    ///         and publishes relevant obstacles
    ///
    void obstacleHandler();


    /* helper functions */

    ///
    /// @brief  loads the robot footprint published by the local costmap
    /// @param  node - NodeHandle to the local costmap
    /// @return footprint polygon as vector
    ///
    std::vector<geometry_msgs::Point> loadRobotFootprint(ros::NodeHandle node);


    ///
    /// @brief  returns the sign of x
    ///
    double sign(double x);
    
    ///
    /// @brief  computes distance between two points
    /// @param  a,b - Points 
    /// @return distance
    ///
    double getDistance2d(geometry_msgs::Point a, geometry_msgs::Point b);

    ///
    /// @brief  checks if obstacle lies already within footprint -> this is ignored due to sensor readings of the hull etc
    /// @param  x_obstacle - x coordinate of obstacle in occupancy grid local costmap
    /// @param  y_obstacle - y coordinate of obstacle in occupancy grid local costmap
    /// @return true if obstacle outside of footprint
    ///
    bool obstacleValid(double x_obstacle, double y_obstacle);

    ///
    /// @brief  stops movement of the robot
    ///
    void stopMovement();

    pthread_mutex_t m_mutex;

    //frames
    std::string global_frame_, robot_frame_;

    //velocity
    geometry_msgs::Vector3 robot_twist_linear_, robot_twist_angular_;
    double v_max_, vtheta_max_;
    double ax_max_, ay_max_, atheta_max_;  

    //obstacle avoidence
    std::vector<geometry_msgs::Point> robot_footprint_;
    double footprint_left_, footprint_right_, footprint_front_, footprint_rear_;
    double footprint_left_initial_, footprint_right_initial_, footprint_front_initial_, footprint_rear_initial_;
    bool costmap_received_;
    nav_msgs::GridCells last_costmap_received_, relevant_obstacles_;
    double influence_radius_, stop_threshold_, obstacle_damping_dist_, use_circumscribed_threshold_;
    double closest_obstacle_dist_, closest_obstacle_angle_;

    // variables for slow down behaviour
    double last_time_;
    double kp_, kv_;
    double vx_last_, vy_last_, vtheta_last_;
    double virt_mass_;

}; //CollisionVelocityFilter

#endif
