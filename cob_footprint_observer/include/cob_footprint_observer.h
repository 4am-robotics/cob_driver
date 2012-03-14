/****************************************************************
 *
 * Copyright (c) 2012
 *
 * Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_navigation
 * ROS package name: cob_footprint_observer
 *                
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *      
 * Author: Matthias Gruhler, email: Matthias.Gruhler@ipa.fraunhofer.de
 *
 * Date of creation: 27.02.2012
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Fraunhofer Institute for Manufacturing 
 *     Engineering and Automation (IPA) nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * @file  cob_footprint_observer.h
 * @brief  observes the footprint of care-o-bot
 *
 * Generates rectangular footprint of care-o-bot depending on setup
 * of tray and arm.
 *
 ****************************************************************/
#ifndef COB_FOOTPRINT_OBSERVER_H
#define COB_FOOTPRINT_OBSERVER_H

//##################
//#### includes ####

// ROS includes
#include <ros/ros.h>
#include <XmlRpc.h>

// message includes
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Vector3.h>

#include <tf/transform_listener.h>

#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include <cob_footprint_observer/GetFootprint.h>
///
/// @class FootprintObserver
/// @brief checks the footprint of care-o-bot and advertises a service to get the adjusted footprint
/// 
///
class FootprintObserver
{
  public:
    ///
    /// @brief  constructor
    ///
    FootprintObserver();
    ///
    /// @brief  destructor
    ///
    ~FootprintObserver();

    ///
    /// @brief  checks the footprint of the robot if it needs to be enlarged due to arm or tray
    ///
    void checkFootprint();
    
    ///
    /// @brief  callback for GetFootprint service
    /// @param  req - request message to service
    /// @param  resp - response message from service
    /// @return service call successfull
    ///
    bool getFootprintCB(cob_footprint_observer::GetFootprint::Request &req, cob_footprint_observer::GetFootprint::Response &resp);

    // public members
    ros::NodeHandle nh_;
    ros::Publisher topic_pub_footprint_;
    ros::ServiceServer srv_get_footprint_;

  private:
    ///
    /// @brief  loads the robot footprint from the costmap node
    /// @param  node - costmap node to check for footprint parameter
    /// @return points of a polygon specifying the footprint
    ///
    std::vector<geometry_msgs::Point> loadRobotFootprint(ros::NodeHandle node);

    ///
    /// @brief  publishes the adjusted footprint as geometry_msgs::StampedPolygon message
    ///
    void publishFootprint();

    ///
    /// @brief  computes the sign of x
    /// @param  x - number
    /// @return sign of x
    ///
    double sign(double x);

    // private members
    std::vector<geometry_msgs::Point> robot_footprint_;
    double footprint_front_initial_, footprint_rear_initial_, footprint_left_initial_, footprint_right_initial_;
    double footprint_front_, footprint_rear_, footprint_left_, footprint_right_;
    tf::TransformListener tf_listener_;
    std::string frames_to_check_;
    std::string robot_base_frame_, farthest_frame_;

    pthread_mutex_t m_mutex;
};

#endif
