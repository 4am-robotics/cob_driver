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

#include <cob_scan_unifier/scan_unifier_node.h>

// Constructor
ScanUnifierNode::ScanUnifierNode()
{
  ROS_DEBUG("Init scan_unifier");

  // Create node handles
  nh_ = ros::NodeHandle();
  pnh_ = ros::NodeHandle("~");

  // Publisher
  topicPub_LaserUnified_ = nh_.advertise<sensor_msgs::LaserScan>("scan_unified", 1);

  getParams();

  // Subscribe to Laserscan topics

  for(int i = 0; i < config_.number_input_scans; i++)
  {
    message_filter_subscribers_.push_back(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, config_.input_scan_topics.at(i), 1));
  }


  // Initialize message_filters::Synchronizer with the right constructor for the choosen number of inputs.

  switch (config_.number_input_scans)
  {
    case 2:
    {
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
      synchronizer2_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(2), *message_filter_subscribers_.at(0), *message_filter_subscribers_.at(1));
      // Set the InterMessageLowerBound to double the period of the laser scans publishing ( 1/{(1/2)*f_laserscans} ).
      synchronizer2_->setInterMessageLowerBound(0, ros::Duration(0.167));
      synchronizer2_->setInterMessageLowerBound(1, ros::Duration(0.167));
      synchronizer2_->registerCallback(boost::bind(&ScanUnifierNode::messageFilterCallback, this, _1, _2));
      break;
    }
    case 3:
    {
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
      synchronizer3_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(3), *message_filter_subscribers_.at(0), *message_filter_subscribers_.at(1), *message_filter_subscribers_.at(2));
      synchronizer3_->setInterMessageLowerBound(0, ros::Duration(0.167));
      synchronizer3_->setInterMessageLowerBound(1, ros::Duration(0.167));
      synchronizer3_->setInterMessageLowerBound(2, ros::Duration(0.167));
      synchronizer3_->registerCallback(boost::bind(&ScanUnifierNode::messageFilterCallback, this, _1, _2, _3));
      break;
    }
    default:
      ROS_ERROR_STREAM(config_.number_input_scans << " topics have been set as input, but scan_unifier doesn't support this.");
      return;
  }

  ros::Duration(1.0).sleep();
}

// Destructor

ScanUnifierNode::~ScanUnifierNode()
{
  delete(synchronizer2_);
  delete(synchronizer3_);
}

/**
 * @function getParams
 * @brief function to load parameters from ros parameter server
 *
 * input: -
 * output: -
 */
void ScanUnifierNode::getParams()
{
  std::vector<std::string> topicList;

  if (pnh_.getParam("input_scans", topicList))
  {
    config_.input_scan_topics = topicList;
    config_.number_input_scans = config_.input_scan_topics.size();
  }
  else
  {
    config_.number_input_scans = 0;
    ROS_ERROR("No parameter input_scans on parameter server!! Scan unifier can not subscribe to any scan topic!");
  }

  if(!pnh_.hasParam("frame"))
  {
    ROS_WARN("No parameter frame on parameter server. Using default value [base_link].");
  }
  pnh_.param<std::string>("frame", frame_, "base_link");
}


void ScanUnifierNode::messageFilterCallback(const sensor_msgs::LaserScan::ConstPtr& scan1, const sensor_msgs::LaserScan::ConstPtr& scan2)
{
  std::vector<sensor_msgs::LaserScan::ConstPtr> current_scans;
  current_scans.push_back(scan1);
  current_scans.push_back(scan2);

  sensor_msgs::LaserScan unified_scan = sensor_msgs::LaserScan();
  if (!unifyLaserScans(current_scans, unified_scan))
  {
    return;
  }

  ROS_DEBUG("Publishing unified scan.");
  topicPub_LaserUnified_.publish(unified_scan);
}

void ScanUnifierNode::messageFilterCallback(const sensor_msgs::LaserScan::ConstPtr& scan1, const sensor_msgs::LaserScan::ConstPtr& scan2, const sensor_msgs::LaserScan::ConstPtr& scan3)
{
  std::vector<sensor_msgs::LaserScan::ConstPtr> current_scans;
  current_scans.push_back(scan1);
  current_scans.push_back(scan2);
  current_scans.push_back(scan3);

  sensor_msgs::LaserScan unified_scan = sensor_msgs::LaserScan();
  if (!unifyLaserScans(current_scans, unified_scan))
  {
    return;
  }

  ROS_DEBUG("Publishing unified scan.");
  topicPub_LaserUnified_.publish(unified_scan);
}

/**
 * @function unifyLaserScans
 * @brief unifie the scan information from all laser scans in vec_laser_struct_
 *
 * input: -
 * output:
 * @param: a laser scan message containing unified information from all scanners
 */
bool ScanUnifierNode::unifyLaserScans(std::vector<sensor_msgs::LaserScan::ConstPtr> current_scans, sensor_msgs::LaserScan &unified_scan)
{
  std::vector<sensor_msgs::PointCloud> vec_cloud;
  vec_cloud.assign(config_.number_input_scans, sensor_msgs::PointCloud());

  if(!current_scans.empty())
  {
    ROS_DEBUG("start converting");
    for(int i=0; i < config_.number_input_scans; i++)
    {
      vec_cloud.at(i).header.stamp = current_scans.at(i)->header.stamp;
      ROS_DEBUG_STREAM("Converting scans to point clouds at index: " << i << ", at time: " << current_scans.at(i)->header.stamp << " now: " << ros::Time::now());
      try
      {
        if (!listener_.waitForTransform(frame_, current_scans.at(i)->header.frame_id,
                                        current_scans.at(i)->header.stamp, ros::Duration(1.0)))
        {
          ROS_WARN_STREAM("Scan unifier skipped scan with " << current_scans.at(i)->header.stamp << " stamp, because of missing tf transform.");
          return false;
        }

        ROS_DEBUG("now project to point_cloud");
        projector_.transformLaserScanToPointCloud(frame_,*current_scans.at(i), vec_cloud.at(i), listener_);
      }
      catch(tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
      }
    }
    ROS_DEBUG("Creating message header");
    unified_scan.header = current_scans.at(0)->header;
    unified_scan.header.frame_id = frame_;
    unified_scan.angle_increment = M_PI/180.0/2.0;
    unified_scan.angle_min = -M_PI + unified_scan.angle_increment*0.01;
    unified_scan.angle_max =  M_PI - unified_scan.angle_increment*0.01;
    unified_scan.time_increment = 0.0;
    unified_scan.scan_time = current_scans.at(0)->scan_time;
    unified_scan.range_min = current_scans.at(0)->range_min;
    unified_scan.range_max = current_scans.at(0)->range_max;
    unified_scan.ranges.resize(round((unified_scan.angle_max - unified_scan.angle_min) / unified_scan.angle_increment) + 1);
    unified_scan.intensities.resize(round((unified_scan.angle_max - unified_scan.angle_min) / unified_scan.angle_increment) + 1);

    // now unify all Scans
    ROS_DEBUG("unify scans");
    for(int j = 0; j < config_.number_input_scans; j++)
    {
      for (unsigned int i = 0; i < vec_cloud.at(j).points.size(); i++)
      {
        const float &x = vec_cloud.at(j).points[i].x;
        const float &y = vec_cloud.at(j).points[i].y;
        const float &z = vec_cloud.at(j).points[i].z;
        //ROS_INFO("Point %f %f %f", x, y, z);
        if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
        {
          ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
          continue;
        }
        double angle = atan2(y, x);// + M_PI/2;
        if (angle < unified_scan.angle_min || angle > unified_scan.angle_max)
        {
          ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, unified_scan.angle_min, unified_scan.angle_max);
          continue;
        }
        int index = std::floor(0.5 + (angle - unified_scan.angle_min) / unified_scan.angle_increment);
        if(index<0 || index>=unified_scan.ranges.size()) continue;

        double range_sq = y*y+x*x;
        //printf ("index xyz( %f %f %f) angle %f index %d range %f\n", x, y, z, angle, index, sqrt(range_sq));
        if( (unified_scan.ranges.at(index) == 0) || (sqrt(range_sq) <= unified_scan.ranges.at(index)) )
        {
          // use the nearest reflection point of all scans for unified scan
          unified_scan.ranges.at(index) = sqrt(range_sq);
          // get respective intensity from point cloud intensity-channel (index 0)
          unified_scan.intensities.at(index) = vec_cloud.at(j).channels.at(0).values.at(i);
        }
      }
    }
  }

  return true;
}

int main(int argc, char** argv)
{
  ROS_DEBUG("scan unifier: start scan unifier node");
  ros::init(argc, argv, "cob_scan_unifier_node");

  ScanUnifierNode scan_unifier_node;

  ros::spin();

  return 0;
}
