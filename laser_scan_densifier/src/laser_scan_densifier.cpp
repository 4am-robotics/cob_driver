/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * Copyright (c) 2011, Ivan Dryanovski, William Morris
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
 *     * Neither the name of the CCNY Robotics Lab nor the names of its
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

#include "laser_scan_densifier/laser_scan_densifier.h"

namespace scan_tools {

LaserScanDensifier::LaserScanDensifier(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  ROS_INFO ("Starting LaserScanDensifier");

  // **** get paramters

  nh_private_.param("step", step_, 2);
  nh_private_.param("mode", mode_, 0);

  if (step_ <= 0)
  {
    ROS_ERROR("LaserScanDensifier has step parameter set to %d, must be > 0! Not initiating subscriptions...", step_);
    exit(-1);
  }

  switch(mode_) {
    case 0: ROS_INFO("LaserScanDensifier started with mode %d: copy data points", mode_);
            break;
    case 1: ROS_INFO("LaserScanDensifier started with mode %d: interpolate data points", mode_);
            break;
    default: ROS_WARN("LaserScanDensifier started with unsupported mode %d. Defaulting to mode 0: copy data points", mode_);
             mode_ = 0;
             break;
  }

  // **** advertise topics

  scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("scan_dense", 1);

  // **** subscribe to laser scan messages

  scan_subscriber_ = nh_.subscribe("scan", 1, &LaserScanDensifier::scanCallback, this);
}

LaserScanDensifier::~LaserScanDensifier ()
{
  ROS_INFO ("Destroying LaserScanDensifier");
}

void LaserScanDensifier::scanCallback (const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  sensor_msgs::LaserScan::Ptr scan_dense;
  scan_dense = boost::make_shared<sensor_msgs::LaserScan>();

  // copy over equal fields

  scan_dense->header          = scan_msg->header;
  scan_dense->range_min       = scan_msg->range_min;
  scan_dense->range_max       = scan_msg->range_max;
  scan_dense->angle_min       = scan_msg->angle_min;
  scan_dense->angle_max       = scan_msg->angle_max;
  scan_dense->angle_increment = scan_msg->angle_increment / step_;
  scan_dense->time_increment  = scan_msg->time_increment;
  scan_dense->scan_time       = scan_msg->scan_time;

  scan_dense->ranges.clear();
  scan_dense->intensities.clear();

  for (size_t i = 0; i < scan_msg->ranges.size()-1; i++)
  {
    switch (mode_) {
      case 0: { //copy data points
        scan_dense->ranges.insert(scan_dense->ranges.end(), step_, scan_msg->ranges[i]);
        scan_dense->intensities.insert(scan_dense->intensities.end(), step_, scan_msg->intensities[i]);
        break;
      }
      case 1: { //interpolate data points
        double delta_range = (scan_msg->ranges[i+1]-scan_msg->ranges[i])/step_;
        double delta_intensities = (scan_msg->intensities[i+1]-scan_msg->intensities[i])/step_;
        for (int k = 0; k < step_; k++) {
          scan_dense->ranges.insert(scan_dense->ranges.end(), 1, scan_msg->ranges[i]+k*delta_range);
          scan_dense->intensities.insert(scan_dense->intensities.end(), 1, scan_msg->intensities[i]+k*delta_intensities);
        }
        break;
      }
    }
  }
  // add angle_max data point
  scan_dense->ranges.push_back(scan_msg->ranges.back());
  scan_dense->intensities.push_back(scan_msg->intensities.back());

  scan_publisher_.publish(scan_dense);
}

} //namespace scan_tools

