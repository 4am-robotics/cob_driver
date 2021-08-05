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


#include <cob_scan_unifier/scan_unifier_node.h>

// Constructor
ScanUnifierNode::ScanUnifierNode()
{
  ROS_DEBUG("Init scan_unifier");

  // Create node handles
  nh_ = ros::NodeHandle();
  pnh_ = ros::NodeHandle("~");

  getParams();

  // Publisher
  topicPub_LaserUnified_ = nh_.advertise<sensor_msgs::LaserScan>("scan_unified", 1);
  if(config_.publish_pointcloud)
  {
    topicPub_PointCloudUnified_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_unified", 1);
  }

  synchronizer2_ = NULL;
  synchronizer3_ = NULL;
  synchronizer4_ = NULL;

  // Subscribe to Laserscan topics

  for(int i = 0; i < config_.number_input_scans; i++)
  {
    message_filter_subscribers_.push_back(
        new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, config_.input_scan_topics[i], 1));
  }


  // Initialize message_filters::Synchronizer with the right constructor for the choosen number of inputs.

  switch (config_.number_input_scans)
  {
    case 2:
    {
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
      synchronizer2_ = new message_filters::Synchronizer<SyncPolicy>(
          SyncPolicy(2), *message_filter_subscribers_.front(), *message_filter_subscribers_[1]);
      // Set the InterMessageLowerBound to double the period of the laser scans publishing ( 1/{(1/2)*f_laserscans} ).
      synchronizer2_->setInterMessageLowerBound(0, ros::Duration(0.167));
      synchronizer2_->setInterMessageLowerBound(1, ros::Duration(0.167));
      synchronizer2_->registerCallback(boost::bind(&ScanUnifierNode::messageFilterCallback, this, _1, _2));
      break;
    }
    case 3:
    {
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
      synchronizer3_ = new message_filters::Synchronizer<SyncPolicy>(
          SyncPolicy(3), *message_filter_subscribers_.front(), *message_filter_subscribers_[1],
          *message_filter_subscribers_[2]);
      synchronizer3_->setInterMessageLowerBound(0, ros::Duration(0.167));
      synchronizer3_->setInterMessageLowerBound(1, ros::Duration(0.167));
      synchronizer3_->setInterMessageLowerBound(2, ros::Duration(0.167));
      synchronizer3_->registerCallback(boost::bind(&ScanUnifierNode::messageFilterCallback, this, _1, _2, _3));
      break;
    }
    case 4:
    {
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
      synchronizer4_ = new message_filters::Synchronizer<SyncPolicy>(
          SyncPolicy(4), *message_filter_subscribers_.front(), *message_filter_subscribers_[1],
          *message_filter_subscribers_[2], *message_filter_subscribers_[3]);
      synchronizer4_->setInterMessageLowerBound(0, ros::Duration(0.167));
      synchronizer4_->setInterMessageLowerBound(1, ros::Duration(0.167));
      synchronizer4_->setInterMessageLowerBound(2, ros::Duration(0.167));
      synchronizer4_->setInterMessageLowerBound(3, ros::Duration(0.167));
      synchronizer4_->registerCallback(boost::bind(&ScanUnifierNode::messageFilterCallback, this, _1, _2, _3, _4));
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
  if(synchronizer2_ != NULL)
    delete(synchronizer2_);
  if(synchronizer3_ != NULL)
    delete(synchronizer3_);
  if(synchronizer4_ != NULL)
    delete(synchronizer4_);
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

  if(!pnh_.hasParam("publish_pointcloud"))
  {
    ROS_WARN("No parameter set for publishing point cloud. Using default value [False].");
    config_.publish_pointcloud = false;
  }
  else
  {
    pnh_.getParam("publish_pointcloud", config_.publish_pointcloud);
  }

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
  publish(unified_scan);
}

void ScanUnifierNode::messageFilterCallback(const sensor_msgs::LaserScan::ConstPtr& scan1,
                                            const sensor_msgs::LaserScan::ConstPtr& scan2,
                                            const sensor_msgs::LaserScan::ConstPtr& scan3)
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
  publish(unified_scan);
}

void ScanUnifierNode::messageFilterCallback(const sensor_msgs::LaserScan::ConstPtr& scan1,
                                            const sensor_msgs::LaserScan::ConstPtr& scan2,
                                            const sensor_msgs::LaserScan::ConstPtr& scan3,
                                            const sensor_msgs::LaserScan::ConstPtr& scan4)
{
  std::vector<sensor_msgs::LaserScan::ConstPtr> current_scans;
  current_scans.push_back(scan1);
  current_scans.push_back(scan2);
  current_scans.push_back(scan3);
  current_scans.push_back(scan4);

  sensor_msgs::LaserScan unified_scan = sensor_msgs::LaserScan();
  if (!unifyLaserScans(current_scans, unified_scan))
  {
    return;
  }
  publish(unified_scan);
}

void ScanUnifierNode::publish(sensor_msgs::LaserScan& unified_scan)
{
  ROS_DEBUG("Publishing unified scan.");
  topicPub_LaserUnified_.publish(unified_scan);
  if(config_.publish_pointcloud)
  {
    sensor_msgs::PointCloud2 unified_pointcloud = sensor_msgs::PointCloud2();
    projector_.transformLaserScanToPointCloud(frame_, unified_scan, unified_pointcloud, listener_);
    topicPub_PointCloudUnified_.publish(unified_pointcloud);
  }
}

/**
 * @function unifyLaserScans
 * @brief unifie the scan information from all laser scans in vec_laser_struct_
 *
 * input: -
 * output:
 * @param: a laser scan message containing unified information from all scanners
 */
bool ScanUnifierNode::unifyLaserScans(const std::vector<sensor_msgs::LaserScan::ConstPtr>& current_scans,
                                      sensor_msgs::LaserScan& unified_scan)
{
  if (vec_cloud_.size() != config_.number_input_scans)
  {
    vec_cloud_.resize(config_.number_input_scans);
  }

  if(!current_scans.empty())
  {
    ROS_DEBUG("start converting");
    for(int i=0; i < config_.number_input_scans; i++)
    {
      vec_cloud_[i].header.stamp = current_scans[i]->header.stamp;
      ROS_DEBUG_STREAM("Converting scans to point clouds at index: "
                       << i << ", at time: " << current_scans[i]->header.stamp << " now: " << ros::Time::now());
      try
      {
        if (!listener_.waitForTransform(frame_, current_scans[i]->header.frame_id, current_scans[i]->header.stamp,
                                        ros::Duration(1.0)))
        {
          ROS_WARN_STREAM("Scan unifier skipped scan with " << current_scans[i]->header.stamp
                                                            << " stamp, because of missing tf transform.");
          return false;
        }

        ROS_DEBUG("now project to pointcloud");
        projector_.transformLaserScanToPointCloud(frame_, *current_scans[i], vec_cloud_[i], listener_);
      }
      catch(tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
      }
    }
    ROS_DEBUG("Creating message header");
    unified_scan.header = current_scans.front()->header;
    unified_scan.header.frame_id = frame_;
    unified_scan.angle_increment = M_PI/180.0/2.0;
    unified_scan.angle_min = -M_PI + unified_scan.angle_increment*0.01;
    unified_scan.angle_max =  M_PI - unified_scan.angle_increment*0.01;
    unified_scan.time_increment = 0.0;
    unified_scan.scan_time = current_scans.front()->scan_time;
    unified_scan.range_min = current_scans.front()->range_min;
    unified_scan.range_max = current_scans.front()->range_max;
    //default values (ranges: range_max, intensities: 0) are used to better reflect the driver behavior
    //there "phantom" data has values > range_max
    //but those values are removed during projection to pointcloud
    unified_scan.ranges.resize(round((unified_scan.angle_max - unified_scan.angle_min) / unified_scan.angle_increment) + 1, unified_scan.range_max);
    unified_scan.intensities.resize(round((unified_scan.angle_max - unified_scan.angle_min) / unified_scan.angle_increment) + 1, 0.0);

    // now unify all Scans
    ROS_DEBUG("unify scans");
    for(int j = 0; j < config_.number_input_scans; j++)
    {
      for (unsigned int i = 0; i < vec_cloud_[j].points.size(); i++)
      {
        const float& x = vec_cloud_[j].points[i].x;
        const float& y = vec_cloud_[j].points[i].y;
        const float& z = vec_cloud_[j].points[i].z;
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
        if ((sqrt(range_sq) <= unified_scan.ranges[index]))
        {
          // use the nearest reflection point of all scans for unified scan
          unified_scan.ranges[index] = sqrt(range_sq);
          // get respective intensity from point cloud intensity-channel (index 0)
          unified_scan.intensities[index] = vec_cloud_[j].channels.front().values[i];
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
