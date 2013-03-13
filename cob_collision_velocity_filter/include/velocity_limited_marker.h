/****************************************************************
 *
 * Copyright (c) 2012 Brno University of Technology (BUT)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_navigation
 * ROS package name: cob_collision_velocity_filter
 *  							
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  		
 * Author: Michal Spanel, email:spanel@fit.vutbr.cz
 *
 * Date of creation: November 2012
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

#pragma once
#ifndef COB_VELOCITY_LIMITED_MARKER_H
#define COB_VELOCITY_LIMITED_MARKER_H

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <visualization_msgs/Marker.h>

namespace cob_collision_velocity_filter
{

///
/// @class CollisionVelocityFilter
/// @brief checks for obstacles in driving direction and stops the robot
///
class VelocityLimitedMarker
{
public:
    ///
    /// @brief  Constructor
    ///
    VelocityLimitedMarker();

    ///
    /// @brief  Destructor
    ///
    ~VelocityLimitedMarker();

    ///
    /// @brief  Creates all directional markers, the method is called from the constructor.
    ///
    void createDirectionalMarkers();

    ///
    /// @brief  Creates all rotational markers, the method is called from the constructor.
    ///
    void createRotationalMarkers();

    ///
    /// @brief  Creates all the markers, the method is called from the constructor.
    ///
    void publishMarkers( double vel_x_desired, 
                    double vel_x_actual, 
                    double vel_y_desired, 
                    double vel_y_actual, 
                    double vel_theta_desired,
                    double vel_theta_actual);

    ///
    /// @brief  Calculates the color for the marker
    ////        based on the absolute difference of velocities.
    ///
    void interpolateColor(double velocity, std_msgs::ColorRGBA& color);

protected:
    // Velocity limited markers
    visualization_msgs::Marker x_pos_marker_, x_neg_marker_, y_pos_marker_, y_neg_marker_;
    visualization_msgs::Marker theta_pos_marker_, theta_neg_marker_;

    // a handle for this node
    ros::NodeHandle nh_;

    // Marker publisher
    ros::Publisher marker_pub_;

    // Is the marker disabled?
    bool disabled_;

    // Robot base frame
    std::string base_frame_;

    // Output topic name
    std::string topic_name_;

    // Marker lifetime
    double lifetime_;

    // Marker z-position
    double z_pos_;

    // last velocities
    double vx_last_, vy_last_, vtheta_last_;
};


}

#endif // COB_VELOCITY_LIMITED_MARKER_H

