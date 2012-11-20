/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2012 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: cob_driver
 * \note
 * ROS package name: cob_camera_sensors
 *
 * \author
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 11/2012
 *
 * \brief
 * Throttles a point cloud stream to a specified rate
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

#include "ros/ros.h"
#include "pluginlib/class_list_macros.h"
#include "nodelet/nodelet.h"
#include "sensor_msgs/PointCloud2.h"


namespace cob_camera_sensors
{
typedef sensor_msgs::PointCloud2 PointCloud;
//typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class Cam3DThrottle : public nodelet::Nodelet
{
public:
  //Constructor
  Cam3DThrottle() :
    max_update_rate_(0),
    sub_counter_(0)
  {
  };

private:
  ros::Time last_update_;
  double max_update_rate_;
  unsigned int sub_counter_;
  ros::NodeHandle nh_;

  virtual void onInit()
  {
    nh_ = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    private_nh.getParam("max_rate", max_update_rate_);

    pub_ = nh_.advertise<PointCloud>("cloud_out", 10, boost::bind(&Cam3DThrottle::connectCB, this, _1), boost::bind(&Cam3DThrottle::disconnectCB, this, _1));
  };

  void callback(const PointCloud::ConstPtr& cloud)
  {
    if (max_update_rate_ > 0.0)
    {
      NODELET_DEBUG("update set to %f", max_update_rate_);
      if ( last_update_ + ros::Duration(1.0/max_update_rate_) > ros::Time::now())
      {
        NODELET_DEBUG("throttle last update at %f skipping", last_update_.toSec());
        return;
      }
    }
    else
      NODELET_DEBUG("update_rate unset continuing");
    last_update_ = ros::Time::now();
    pub_.publish(cloud);
  }

  void connectCB(const ros::SingleSubscriberPublisher& pub)
  {
    sub_counter_++;
    if(sub_counter_ == 1)
      sub_ = nh_.subscribe<PointCloud>("cloud_in", 10, &Cam3DThrottle::callback, this);
  }

  void disconnectCB(const ros::SingleSubscriberPublisher& pub)
  {
    sub_counter_--;
    if(sub_counter_ == 0)
      sub_.shutdown();
  }

  ros::Publisher pub_;
  ros::Subscriber sub_;

};


PLUGINLIB_DECLARE_CLASS(srs_env_model, Cam3DThrottle, cob_camera_sensors::Cam3DThrottle, nodelet::Nodelet);
}

