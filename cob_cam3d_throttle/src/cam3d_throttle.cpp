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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


namespace cob_cam3d_throttle
{
typedef sensor_msgs::PointCloud2 tPointCloud;
typedef sensor_msgs::Image tImage;
typedef sensor_msgs::CameraInfo tCameraInfo;
//typedef message_filters::sync_policies::ApproximateTime<tPointCloud, tImage, tCameraInfo, tImage, tPointCloud> tSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<tPointCloud, tImage, tCameraInfo> tSyncPolicy;

class Cam3DThrottle : public nodelet::Nodelet
{
public:
  //Constructor
  Cam3DThrottle() :
    max_update_rate_(0),
    sub_counter_(0),
    sync_(tSyncPolicy(3))
  {
  };

private:
  virtual void onInit()
  {
    nh_ = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    private_nh.getParam("max_rate", max_update_rate_);

    rgb_cloud_pub_ = nh_.advertise<tPointCloud>("rgb_cloud_out", 1, boost::bind(&Cam3DThrottle::connectCB, this, _1), boost::bind(&Cam3DThrottle::disconnectCB, this, _1));
    rgb_image_pub_ = nh_.advertise<tImage>("rgb_image_out", 1, boost::bind(&Cam3DThrottle::connectCB, this, _1), boost::bind(&Cam3DThrottle::disconnectCB, this, _1));
    rgb_caminfo_pub_ = nh_.advertise<tCameraInfo>("rgb_caminfo_out", 1, boost::bind(&Cam3DThrottle::connectCB, this, _1), boost::bind(&Cam3DThrottle::disconnectCB, this, _1));
    depth_image_pub_ = nh_.advertise<tImage>("depth_image_out", 1, boost::bind(&Cam3DThrottle::connectCB, this, _1), boost::bind(&Cam3DThrottle::disconnectCB, this, _1));
    cloud_pub_ = nh_.advertise<tPointCloud>("cloud_out", 1, boost::bind(&Cam3DThrottle::connectCB, this, _1), boost::bind(&Cam3DThrottle::disconnectCB, this, _1));

    last_update_ = ros::Time::now();
    //sync_.connectInput(rgb_cloud_sub_, rgb_image_sub_, rgb_caminfo_sub_, depth_image_sub_, cloud_sub_);
    //sync_.registerCallback(boost::bind(&Cam3DThrottle::callback, this, _1, _2, _3, _4, _5));
    sync_.connectInput(rgb_cloud_sub_, rgb_image_sub_, rgb_caminfo_sub_);
    sync_.registerCallback(boost::bind(&Cam3DThrottle::callback, this, _1, _2, _3));
  };

/*  void callback(const tPointCloud::ConstPtr& rgb_cloud,
                const tImage::ConstPtr& rgb_image,
                const tCameraInfo::ConstPtr& rgb_caminfo,
                const tImage::ConstPtr& depth_image,
                const tPointCloud::ConstPtr& cloud
                )
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
    rgb_cloud_pub_.publish(rgb_cloud);
    rgb_image_pub_.publish(rgb_image);
    rgb_caminfo_pub_.publish(rgb_caminfo);
    depth_image_pub_.publish(depth_image);
    cloud_pub_.publish(cloud);
  }*/
  
  void callback(const tPointCloud::ConstPtr& rgb_cloud,
                const tImage::ConstPtr& rgb_image,
                const tCameraInfo::ConstPtr& rgb_caminfo
                )
  {
    
    if (max_update_rate_ > 0.0)
    {
      NODELET_DEBUG("update set to %f", max_update_rate_);
      //std::cout << "update rate: " << max_update_rate_ << std::endl;
      if ( last_update_ + ros::Duration(1.0/max_update_rate_) > ros::Time::now())
      {
      	//std::cout << "last_update[s] " << last_update_.toSec() << std::endl;
        NODELET_DEBUG("throttle last update at %f skipping", last_update_.toSec());
        return;
      }
    }
    else
    {
      NODELET_DEBUG("update_rate unset continuing");
    }
    //std::cout << "publishing" << std::endl;
    last_update_ = ros::Time::now();
    rgb_cloud_pub_.publish(rgb_cloud);
    rgb_image_pub_.publish(rgb_image);
    rgb_caminfo_pub_.publish(rgb_caminfo);
  }

  void connectCB(const ros::SingleSubscriberPublisher& pub)
  {
    sub_counter_++;
    //std::cout << "connectCB: "<<  sub_counter_ << std::endl;
    if(sub_counter_ == 1)
    {
      ROS_DEBUG("connecting");
      rgb_cloud_sub_.subscribe(nh_, "rgb_cloud_in", 1);
      rgb_image_sub_.subscribe(nh_, "rgb_image_in", 1);
      rgb_caminfo_sub_.subscribe(nh_, "rgb_caminfo_in", 1);
//      depth_image_sub_.subscribe(nh_, "depth_image_in", 1);
//      cloud_sub_.subscribe(nh_, "cloud_in", 1);
    }
  }

  void disconnectCB(const ros::SingleSubscriberPublisher& pub)
  {
    sub_counter_--;
    if(sub_counter_ == 0)
    {
      ROS_DEBUG("disconnecting");
      rgb_cloud_sub_.unsubscribe();
      rgb_image_sub_.unsubscribe();
      rgb_caminfo_sub_.unsubscribe();
//      depth_image_sub_.unsubscribe();
//      cloud_sub_.unsubscribe();
    }
  }

  ros::Time last_update_;
  double max_update_rate_;
  unsigned int sub_counter_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Subscriber sub2_;
  ros::Publisher rgb_cloud_pub_, rgb_image_pub_, rgb_caminfo_pub_, depth_image_pub_, cloud_pub_;

  message_filters::Subscriber<tPointCloud> rgb_cloud_sub_;
  message_filters::Subscriber<tImage> rgb_image_sub_;
  message_filters::Subscriber<tCameraInfo> rgb_caminfo_sub_;
  message_filters::Subscriber<tImage> depth_image_sub_;
  message_filters::Subscriber<tPointCloud> cloud_sub_;
  message_filters::Synchronizer<tSyncPolicy> sync_;

};


PLUGINLIB_DECLARE_CLASS(cob_cam3d_throttle, Cam3DThrottle, cob_cam3d_throttle::Cam3DThrottle, nodelet::Nodelet);
}

