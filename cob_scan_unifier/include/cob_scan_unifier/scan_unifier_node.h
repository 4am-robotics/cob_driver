/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2014 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Repository name: cob_navigation
* \note
* ROS package name: cob_scan_unifier
*
* \author
* Author: Florian Mirus, email:florian.mirus@ipa.fhg.de
* \author
* Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
*
* \date Date of creation: January 2011
* \date Last Modification: June 2014
*
* \brief
* Takes in several laser scans and publishes them as a single one
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/

#ifndef SCAN_UNIFIER_NODE_H
#define SCAN_UNIFIER_NODE_H

//##################
//#### includes ####

// standard includes
#include <pthread.h>
#include <XmlRpc.h>
#include <math.h>

// ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ROS message includes
#include <sensor_msgs/LaserScan.h>


//####################
//#### node class ####
class ScanUnifierNode
{
  private:
    /** @struct config_struct
     *  @brief This structure holds configuration parameters
     *  @var config_struct::number_input_scans
     *  Member 'number_input_scans' contains number of scanners to subscribe to
     *  @var config_struct::loop_rate
     *  Member 'loop_rate' contains the loop rate of the ros node
     *  @var config_struct::input_scan_topics
     *  Member 'input_scan_topics' contains the names of the input scan topics
     */
    struct config_struct{
      int number_input_scans;
      std::vector<std::string> input_scan_topics;
    };

    config_struct config_;

    std::string frame_;

    std::vector<message_filters::Subscriber<sensor_msgs::LaserScan>* > message_filter_subscribers_;

    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> >* synchronizer2_;
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan> >* synchronizer3_;

    void messageFilterCallback(const sensor_msgs::LaserScan::ConstPtr& first_scanner, const sensor_msgs::LaserScan::ConstPtr& second_scanner);
    void messageFilterCallback(const sensor_msgs::LaserScan::ConstPtr& first_scanner, const sensor_msgs::LaserScan::ConstPtr& second_scanner, const sensor_msgs::LaserScan::ConstPtr& third_scanner);

  public:

    // constructor
    ScanUnifierNode();

    // destructor
    ~ScanUnifierNode();

    /* ----------------------------------- */
    /* --------- ROS Variables ----------- */
    /* ----------------------------------- */

    // create node handles
    ros::NodeHandle nh_, pnh_;

    // declaration of ros publishers
    ros::Publisher topicPub_LaserUnified_;

    // tf listener
    tf::TransformListener listener_;

    // laser geometry projector
    laser_geometry::LaserProjection projector_;

    /* ----------------------------------- */
    /* ----------- functions ------------- */
    /* ----------------------------------- */

    /**
     * @function getParams
     * @brief function to load parameters from ros parameter server
     *
     * input: -
     * output: -
     */
    void getParams();

    /**
     * @function unifieLaserScans
     * @brief unifie the scan information from all laser scans in vec_laser_struct_
     *
     * input: -
     * output:
     * @param: a laser scan message containing unified information from all scanners
     */
    bool unifyLaserScans(std::vector<sensor_msgs::LaserScan::ConstPtr> current_scans, sensor_msgs::LaserScan &unified_scan);

};
#endif
