/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Modified to work with RGB point clouds by Michal Spanel, Robo@FIT, BUT
 */

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
  Cam3DThrottle(): max_update_rate_(0)
  {
  };

private:
  ros::Time last_update_;
  double max_update_rate_;
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    private_nh.getParam("max_rate", max_update_rate_);

    pub_ = nh.advertise<PointCloud>("cloud_out", 10);
    sub_ = nh.subscribe<PointCloud>("cloud_in", 10, &Cam3DThrottle::callback, this);
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

  ros::Publisher pub_;
  ros::Subscriber sub_;

};


PLUGINLIB_DECLARE_CLASS(cob_camera_sensors, Cam3DThrottle, cob_camera_sensors::Cam3DThrottle, nodelet::Nodelet);
}

