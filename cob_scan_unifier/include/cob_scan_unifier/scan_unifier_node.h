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
#include <sensor_msgs/PointCloud2.h>

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
      bool publish_pointcloud;
    };

    config_struct config_;

    std::string frame_;

    std::vector<message_filters::Subscriber<sensor_msgs::LaserScan>* > message_filter_subscribers_;

    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,
                                                                                  sensor_msgs::LaserScan> >* synchronizer2_;
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,
                                                                                  sensor_msgs::LaserScan,
                                                                                  sensor_msgs::LaserScan> >* synchronizer3_;
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,
                                                                                  sensor_msgs::LaserScan,
                                                                                  sensor_msgs::LaserScan,
                                                                                  sensor_msgs::LaserScan> >* synchronizer4_;

    void messageFilterCallback(const sensor_msgs::LaserScan::ConstPtr& first_scanner,
                               const sensor_msgs::LaserScan::ConstPtr& second_scanner);
    void messageFilterCallback(const sensor_msgs::LaserScan::ConstPtr& first_scanner,
                               const sensor_msgs::LaserScan::ConstPtr& second_scanner,
                               const sensor_msgs::LaserScan::ConstPtr& third_scanner);
    void messageFilterCallback(const sensor_msgs::LaserScan::ConstPtr& first_scanner,
                               const sensor_msgs::LaserScan::ConstPtr& second_scanner,
                               const sensor_msgs::LaserScan::ConstPtr& third_scanner,
                               const sensor_msgs::LaserScan::ConstPtr& fourth_scanner);

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
    ros::Publisher topicPub_PointCloudUnified_;

    // tf listener
    tf::TransformListener listener_;

    // laser geometry projector
    laser_geometry::LaserProjection projector_;

    std::vector<sensor_msgs::PointCloud> vec_cloud_;

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
    void publish(sensor_msgs::LaserScan& unified_scan);
    bool unifyLaserScans(const std::vector<sensor_msgs::LaserScan::ConstPtr>& current_scans,
                         sensor_msgs::LaserScan& unified_scan);
};
#endif
