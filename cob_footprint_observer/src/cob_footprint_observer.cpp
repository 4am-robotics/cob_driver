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
 * @file  cob_footprint_observer.cpp
 * @brief  observes the footprint of care-o-bot
 *
 * Generates rectangular footprint of care-o-bot depending on setup of
 * arm and tray
 *
 ****************************************************************/
#include <cob_footprint_observer.h>

// Constructor
FootprintObserver::FootprintObserver()
{
  nh_ = ros::NodeHandle("~");

  m_mutex = PTHREAD_MUTEX_INITIALIZER;

  // publish footprint
  topic_pub_footprint_ = nh_.advertise<geometry_msgs::PolygonStamped>("adjusted_footprint",1);
  
  // advertise service
  srv_get_footprint_ = nh_.advertiseService("/get_footprint", &FootprintObserver::getFootprintCB, this);

  // read footprint_source parameter
  std::string footprint_source; 
  if(!nh_.hasParam("footprint_source")) ROS_WARN("Checking default location (/local_costmap_node/costmap) for initial footprint parameter.");
  nh_.param("footprint_source", footprint_source, std::string("/local_costmap_node/costmap"));
 
  // node handle to get footprint from parameter server
  ros::NodeHandle footprint_source_nh_(footprint_source); 	
  
  // load the robot footprint from the parameter server if its available in the local costmap namespace
  robot_footprint_ = loadRobotFootprint(footprint_source_nh_);
  if(robot_footprint_.size() > 4) 
    ROS_WARN("You have set more than 4 points as robot_footprint, cob_footprint_observer can deal only with rectangular footprints so far!");
  
  // get the frames for which to check the footprint
  if(!nh_.hasParam("frames_to_check")) ROS_WARN("No frames to check for footprint observer. Only using initial footprint!");
  nh_.param("frames_to_check", frames_to_check_, std::string(""));
  
  if(!nh_.hasParam("robot_base_frame")) ROS_WARN("No parameter robot_base_frame on parameter server. Using default [/base_link].");
  nh_.param("robot_base_frame", robot_base_frame_, std::string("/base_link"));
  
  if(!nh_.hasParam("farthest_frame")) ROS_WARN("No parameter farthest_frame on parameter server. Using default [/base_link].");
  nh_.param("farthest_frame", farthest_frame_, std::string("/base_link"));

  // wait until transform to from robot_base_frame_ to farthest_frame_ is available
  ros::Time last_error = ros::Time::now();
  std::string tf_error;
  while(!tf_listener_.waitForTransform(robot_base_frame_, farthest_frame_, ros::Time(), ros::Duration(0.1), ros::Duration(0.01), &tf_error)) {
    ros::spinOnce();
    if(last_error + ros::Duration(5.0) < ros::Time::now()){
      ROS_WARN("Waiting on transform from %s to %s to become available before running cob_footprint_observer, tf errror: %s", robot_base_frame_.c_str(), farthest_frame_.c_str(), tf_error.c_str());
      last_error = ros::Time::now();
    }
  }
}

// Destructor
FootprintObserver::~FootprintObserver()
{}

// GetFootprint Service callback
bool FootprintObserver::getFootprintCB(cob_footprint_observer::GetFootprint::Request &req, cob_footprint_observer::GetFootprint::Response &resp)
{
  // create Polygon message for service call
  geometry_msgs::PolygonStamped footprint_poly;
  footprint_poly.header.frame_id = robot_base_frame_;
  footprint_poly.header.stamp = ros::Time::now();
  
  footprint_poly.polygon.points.resize(robot_footprint_.size());
  for(unsigned int i=0; i<robot_footprint_.size(); ++i) {
    footprint_poly.polygon.points[i].x = robot_footprint_[i].x;
    footprint_poly.polygon.points[i].y = robot_footprint_[i].y;
    footprint_poly.polygon.points[i].z = robot_footprint_[i].z;   
  }

  resp.footprint = footprint_poly;
  resp.success.data = true;

  return true;
}

// load robot footprint from costmap_2d_ros to keep same footprint format
std::vector<geometry_msgs::Point> FootprintObserver::loadRobotFootprint(ros::NodeHandle node){
  std::vector<geometry_msgs::Point> footprint;
  geometry_msgs::Point pt;
  double padding;

  std::string padding_param, footprint_param;
  if(!node.searchParam("footprint_padding", padding_param))
    padding = 0.01;
  else
    node.param(padding_param, padding, 0.01);

  //grab the footprint from the parameter server if possible
  XmlRpc::XmlRpcValue footprint_list;
  std::string footprint_string;
  std::vector<std::string> footstring_list;
  if(node.searchParam("footprint", footprint_param)){
    node.getParam(footprint_param, footprint_list);
    if(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
      footprint_string = std::string(footprint_list);

      //if there's just an empty footprint up there, return
      if(footprint_string == "[]" || footprint_string == "")
        return footprint;

      boost::erase_all(footprint_string, " ");

      boost::char_separator<char> sep("[]");
      boost::tokenizer<boost::char_separator<char> > tokens(footprint_string, sep);
      footstring_list = std::vector<std::string>(tokens.begin(), tokens.end());
    }
    //make sure we have a list of lists
    if(!(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeArray && footprint_list.size() > 2) && !(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString && footstring_list.size() > 5)){
      ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s", footprint_param.c_str(), std::string(footprint_list).c_str());
      throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
    }

    if(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for(int i = 0; i < footprint_list.size(); ++i){
        //make sure we have a list of lists of size 2
        XmlRpc::XmlRpcValue point = footprint_list[i];
        if(!(point.getType() == XmlRpc::XmlRpcValue::TypeArray && point.size() == 2)){
          ROS_FATAL("The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
          throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
        }

        //make sure that the value we're looking at is either a double or an int
        if(!(point[0].getType() == XmlRpc::XmlRpcValue::TypeInt || point[0].getType() == XmlRpc::XmlRpcValue::TypeDouble)){
          ROS_FATAL("Values in the footprint specification must be numbers");
          throw std::runtime_error("Values in the footprint specification must be numbers");
        }
        pt.x = point[0].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(point[0]) : (double)(point[0]);
        pt.x += sign(pt.x) * padding;

        //make sure that the value we're looking at is either a double or an int
        if(!(point[1].getType() == XmlRpc::XmlRpcValue::TypeInt || point[1].getType() == XmlRpc::XmlRpcValue::TypeDouble)){
          ROS_FATAL("Values in the footprint specification must be numbers");
          throw std::runtime_error("Values in the footprint specification must be numbers");
        }
        pt.y = point[1].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(point[1]) : (double)(point[1]);
        pt.y += sign(pt.y) * padding;

        footprint.push_back(pt);

        node.deleteParam(footprint_param);
        std::ostringstream oss;
        bool first = true;
        BOOST_FOREACH(geometry_msgs::Point p, footprint) {
          if(first) {
            oss << "[[" << p.x << "," << p.y << "]";
            first = false;
          }
          else {
            oss << ",[" << p.x << "," << p.y << "]";
          }
        }
        oss << "]";
        node.setParam(footprint_param, oss.str().c_str());
        node.setParam("footprint", oss.str().c_str());
      }
    }

    else if(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
      std::vector<geometry_msgs::Point> footprint_spec;
      bool valid_foot = true;
      BOOST_FOREACH(std::string t, footstring_list) {
        if( t != "," ) {
          boost::erase_all(t, " ");
          boost::char_separator<char> pt_sep(",");
          boost::tokenizer<boost::char_separator<char> > pt_tokens(t, pt_sep);
          std::vector<std::string> point(pt_tokens.begin(), pt_tokens.end());

          if(point.size() != 2) {
            ROS_WARN("Each point must have exactly 2 coordinates");
            valid_foot = false;
            break;
          }

          std::vector<double>tmp_pt;
          BOOST_FOREACH(std::string p, point) {
            std::istringstream iss(p);
            double temp;
            if(iss >> temp) {
              tmp_pt.push_back(temp);
            }
            else {
              ROS_WARN("Each coordinate must convert to a double.");
              valid_foot = false;
              break;
            }
          }
          if(!valid_foot)
            break;

          geometry_msgs::Point pt;
          pt.x = tmp_pt[0];
          pt.y = tmp_pt[1];

          footprint_spec.push_back(pt);
        }
      }
      if (valid_foot) {
        footprint = footprint_spec;
        node.setParam("footprint", footprint_string);
      }
      else {
        ROS_FATAL("This footprint is not vaid it must be specified as a list of lists with at least 3 points, you specified %s", footprint_string.c_str());
        throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
      }
    }
  }

  footprint_right_ = 0.0f; footprint_left_ = 0.0f; footprint_front_ = 0.0f; footprint_rear_ = 0.0f;
  //extract rectangular borders for simplifying:
  for(unsigned int i=0; i<footprint.size(); i++) {
    if(footprint[i].x > footprint_front_) footprint_front_ = footprint[i].x;
    if(footprint[i].x < footprint_rear_) footprint_rear_ = footprint[i].x;
    if(footprint[i].y > footprint_left_) footprint_left_ = footprint[i].y;
    if(footprint[i].y < footprint_right_) footprint_right_ = footprint[i].y;
  }
  ROS_DEBUG("Extracted rectangular footprint for cob_footprint_observer: Front: %f, Rear %f, Left: %f, Right %f", footprint_front_, footprint_rear_, footprint_left_, footprint_right_);

  if ( fabs(footprint_front_ - footprint_rear_) == 0.0 || fabs(footprint_right_ - footprint_left_) == 0.0){
    ROS_WARN("Footprint has no physical dimension!");
  }

  // save initial footprint
  footprint_front_initial_ = footprint_front_;
  footprint_rear_initial_ = footprint_rear_;
  footprint_left_initial_ = footprint_left_;
  footprint_right_initial_ = footprint_right_;

  return footprint;
}

// checks if footprint has to be adjusted and does so if necessary
void FootprintObserver::checkFootprint(){
  // check each frame
  std::string frame;
  std::stringstream ss;
  ss << frames_to_check_; 

  double x_rear, x_front, y_left, y_right;
  x_front = footprint_front_initial_;
  x_rear = footprint_rear_initial_;
  y_left = footprint_left_initial_;
  y_right = footprint_right_initial_;
 
  while(ss >> frame){
    // get transform between robot base frame and frame
    if(tf_listener_.canTransform(robot_base_frame_, frame, ros::Time(0))) {
      tf::StampedTransform transform;
      tf_listener_.lookupTransform(robot_base_frame_, frame, ros::Time(0), transform);
      
      tf::Vector3 frame_position = transform.getOrigin();

      // check if frame position is outside of current footprint
      if(frame_position.x() > x_front) x_front = frame_position.x();
      if(frame_position.x() < x_rear) x_rear = frame_position.x();
      if(frame_position.y() > y_left) y_left = frame_position.y();
      if(frame_position.y() < y_right) y_right = frame_position.y();
    }
  }
  
  pthread_mutex_lock(&m_mutex);
  // adjust footprint
  footprint_front_ = x_front; 
  footprint_rear_ = x_rear;
  footprint_left_ = y_left;
  footprint_right_ = y_right;
  pthread_mutex_unlock(&m_mutex);

  // create new footprint vector
  geometry_msgs::Point point;
  std::vector<geometry_msgs::Point> points;

  point.x = footprint_front_;
  point.y = footprint_left_;
  point.z = 0;
  points.push_back(point);
  point.y = footprint_right_;
  points.push_back(point);
  point.x = footprint_rear_;
  points.push_back(point);
  point.y = footprint_left_;
  points.push_back(point);

  pthread_mutex_lock(&m_mutex);
  robot_footprint_ = points;
  pthread_mutex_unlock(&m_mutex);

  // publish the adjusted footprint
  publishFootprint();
}

// publishes the adjusted footprint
void FootprintObserver::publishFootprint(){

  // create Polygon message 
  geometry_msgs::PolygonStamped footprint_poly;
  footprint_poly.header.frame_id = robot_base_frame_;
  footprint_poly.header.stamp = ros::Time::now();
  
  footprint_poly.polygon.points.resize(robot_footprint_.size());
  for(unsigned int i=0; i<robot_footprint_.size(); ++i) {
    footprint_poly.polygon.points[i].x = robot_footprint_[i].x;
    footprint_poly.polygon.points[i].y = robot_footprint_[i].y;
    footprint_poly.polygon.points[i].z = robot_footprint_[i].z;   
  }

  // publish adjusted footprint
  topic_pub_footprint_.publish(footprint_poly);

}

// compute the sign of x
double FootprintObserver::sign(double x){
  if(x >= 0.0f) return 1.0f;
  else return -1.0f;
}

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
  // initialize ROS, specify name of node
  ros::init(argc,argv,"footprint_observer");

  // create observer
  FootprintObserver footprintObserver;

  // run footprint observer periodically until node has been shut down
  ros::Rate loop_rate(1); // Hz
  while(footprintObserver.nh_.ok()){
    footprintObserver.checkFootprint();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
};
