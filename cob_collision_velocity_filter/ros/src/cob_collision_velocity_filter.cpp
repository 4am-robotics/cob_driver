/****************************************************************
 *
 * Copyright (c) 2012
 *
 * Fraunhofer Institute for Manufacturing Engineering  
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_navigation
 * ROS package name: cob_collision_velocity_filter
 *  							
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  		
 * Author: Matthias Gruhler, email:Matthias.Gruhler@ipa.fhg.de
 *
 * Date of creation: February 2012
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *  	 notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *  	 notice, this list of conditions and the following disclaimer in the
 *  	 documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Fraunhofer Institute for Manufacturing 
 *  	 Engineering and Automation (IPA) nor the names of its
 *  	 contributors may be used to endorse or promote products derived from
 *  	 this software without specific prior written permission.
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
 ****************************************************************/
#include <cob_collision_velocity_filter.h>

// Constructor
CollisionVelocityFilter::CollisionVelocityFilter()
{
  // create node handle
  nh_ = ros::NodeHandle("~");

  m_mutex = PTHREAD_MUTEX_INITIALIZER;

  // node handle to get footprint from parameter server
  std::string costmap_parameter_source;
  if(!nh_.hasParam("costmap_parameter_source")) ROS_WARN("Checking default source [/local_costmap_node/costmap] for costmap parameters");
  nh_.param("costmap_parameter_source",costmap_parameter_source, std::string("/local_costmap_node/costmap"));

  ros::NodeHandle local_costmap_nh_(costmap_parameter_source); 	

  // implementation of topics to publish (command for base and list of relevant obstacles)
  topic_pub_command_ = nh_.advertise<geometry_msgs::Twist>("command", 1);
  topic_pub_relevant_obstacles_ = nh_.advertise<nav_msgs::GridCells>("relevant_obstacles", 1);

  // subscribe to twist-movement of teleop 
  joystick_velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("teleop_twist", 1, boost::bind(&CollisionVelocityFilter::joystickVelocityCB, this, _1));
  // subscribe to the costmap to receive inflated cells
  obstacles_sub_ = nh_.subscribe<nav_msgs::GridCells>("obstacles", 1, boost::bind(&CollisionVelocityFilter::obstaclesCB, this, _1));

  // create service client
  srv_client_get_footprint_ = nh_.serviceClient<cob_footprint_observer::GetFootprint>("/get_footprint");

  // create Timer and call getFootprint Service periodically
  double footprint_update_frequency;
  if(!nh_.hasParam("footprint_update_frequency")) ROS_WARN("Used default parameter for footprint_update_frequency [1.0 Hz].");
  nh_.param("footprint_update_frequency",footprint_update_frequency,1.0);
  get_footprint_timer_ = nh_.createTimer(ros::Duration(1/footprint_update_frequency), boost::bind(&CollisionVelocityFilter::getFootprintServiceCB, this, _1));

  // read parameters from parameter server
  // parameters from costmap
  if(!local_costmap_nh_.hasParam(costmap_parameter_source+"/global_frame")) ROS_WARN("Used default parameter for global_frame [/base_link]");
  local_costmap_nh_.param(costmap_parameter_source+"/global_frame", global_frame_, std::string("/base_link"));

  if(!local_costmap_nh_.hasParam(costmap_parameter_source+"/robot_base_frame")) ROS_WARN("Used default parameter for robot_frame [/base_link]");
  local_costmap_nh_.param(costmap_parameter_source+"/robot_base_frame", robot_frame_, std::string("/base_link"));

  if(!nh_.hasParam("influence_radius")) ROS_WARN("Used default parameter for influence_radius [1.5 m]");
  nh_.param("influence_radius", influence_radius_, 1.5);
  closest_obstacle_dist_ = influence_radius_;
  closest_obstacle_angle_ = 0.0;

  // parameters for obstacle avoidence and velocity adjustment
  if(!nh_.hasParam("stop_threshold")) ROS_WARN("Used default parameter for stop_threshold [0.1 m]");
  nh_.param("stop_threshold", stop_threshold_, 0.10);

  if(!nh_.hasParam("obstacle_damping_dist")) ROS_WARN("Used default parameter for obstacle_damping_dist [5.0 m]");
  nh_.param("obstacle_damping_dist", obstacle_damping_dist_, 5.0);
  if(obstacle_damping_dist_ < stop_threshold_) {
    ROS_WARN("obstacle_damping_dist < stop_threshold => robot will stop without decceleration!");
  }

  if(!nh_.hasParam("use_circumscribed_threshold")) ROS_WARN("Used default parameter for use_circumscribed_threshold_ [0.2 rad/s]");
  nh_.param("use_circumscribed_threshold", use_circumscribed_threshold_, 0.20);

  if(!nh_.hasParam("pot_ctrl_vmax")) ROS_WARN("Used default parameter for pot_ctrl_vmax [0.6]");
  nh_.param("pot_ctrl_vmax", v_max_, 0.6);

  if(!nh_.hasParam("pot_ctrl_vtheta_max")) ROS_WARN("Used default parameter for pot_ctrl_vtheta_max [0.8]");
  nh_.param("pot_ctrl_vtheta_max", vtheta_max_, 0.8);

  if(!nh_.hasParam("pot_ctrl_kv")) ROS_WARN("Used default parameter for pot_ctrl_kv [1.0]");
  nh_.param("pot_ctrl_kv", kv_, 1.0);

  if(!nh_.hasParam("pot_ctrl_kp")) ROS_WARN("Used default parameter for pot_ctrl_kp [2.0]");
  nh_.param("pot_ctrl_kp", kp_, 2.0);

  if(!nh_.hasParam("pot_ctrl_virt_mass")) ROS_WARN("Used default parameter for pot_ctrl_virt_mass [0.8]");
  nh_.param("pot_ctrl_virt_mass", virt_mass_, 0.8);

  //load the robot footprint from the parameter server if its available in the local costmap namespace
  robot_footprint_ = loadRobotFootprint(local_costmap_nh_);
  if(robot_footprint_.size() > 4) 
    ROS_WARN("You have set more than 4 points as robot_footprint, cob_collision_velocity_filter can deal only with rectangular footprints so far!");

  // try to geht the max_acceleration values from the parameter server
  if(!nh_.hasParam("max_acceleration")) ROS_WARN("Used default parameter for max_acceleration [0.5, 0.5, 0.7]");
  XmlRpc::XmlRpcValue max_acc;
  if(nh_.getParam("max_acceleration", max_acc)) {
    ROS_ASSERT(max_acc.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ax_max_ = (double)max_acc[0];
    ay_max_ = (double)max_acc[1];
    atheta_max_ = (double)max_acc[2];
  } else {
    ax_max_ = 0.5;
    ay_max_ = 0.5;
    atheta_max_ = 0.7;
  }

  last_time_ = ros::Time::now().toSec();
  vx_last_ = 0.0;
  vy_last_ = 0.0;
  vtheta_last_ = 0.0;
} 

// Destructor
CollisionVelocityFilter::~CollisionVelocityFilter(){}

// joystick_velocityCB reads twist command from joystick
void CollisionVelocityFilter::joystickVelocityCB(const geometry_msgs::Twist::ConstPtr &twist){
  pthread_mutex_lock(&m_mutex);

  robot_twist_linear_ = twist->linear;
  robot_twist_angular_ = twist->angular;

  pthread_mutex_unlock(&m_mutex);

  // check for relevant obstacles
  obstacleHandler();
  // stop if we are about to run in an obstacle
  performControllerStep();

}

// obstaclesCB reads obstacles from costmap
void CollisionVelocityFilter::obstaclesCB(const nav_msgs::GridCells::ConstPtr &obstacles){
  pthread_mutex_lock(&m_mutex);

  if(obstacles->cells.size()!=0) costmap_received_ = true;
  last_costmap_received_ = * obstacles;

  if(stop_threshold_ < obstacles->cell_width / 2.0f || stop_threshold_ < obstacles->cell_height / 2.0f)
    ROS_WARN("You specified a stop_threshold that is smaller than resolution of received costmap!");

  pthread_mutex_unlock(&m_mutex);
}

// timer callback for periodically checking footprint
void CollisionVelocityFilter::getFootprintServiceCB(const ros::TimerEvent&) 
{
  cob_footprint_observer::GetFootprint srv = cob_footprint_observer::GetFootprint();
  // check if service is reachable
  if (srv_client_get_footprint_.call(srv))
  {
    // adjust footprint
    geometry_msgs::PolygonStamped footprint_poly = srv.response.footprint;
    std::vector<geometry_msgs::Point> footprint;
    geometry_msgs::Point pt;

    for(unsigned int i=0; i<footprint_poly.polygon.points.size(); ++i) {
      pt.x = footprint_poly.polygon.points[i].x;
      pt.y = footprint_poly.polygon.points[i].y;
      pt.z = footprint_poly.polygon.points[i].z;
      footprint.push_back(pt);
    }

    pthread_mutex_lock(&m_mutex);

    footprint_front_ = footprint_front_initial_;
    footprint_rear_ = footprint_rear_initial_;
    footprint_left_ = footprint_left_initial_;
    footprint_right_ = footprint_right_initial_;

    robot_footprint_ = footprint;
    for(unsigned int i=0; i<footprint.size(); i++) {
      if(footprint[i].x > footprint_front_) footprint_front_ = footprint[i].x;
      if(footprint[i].x < footprint_rear_) footprint_rear_ = footprint[i].x;
      if(footprint[i].y > footprint_left_) footprint_left_ = footprint[i].y;
      if(footprint[i].y < footprint_right_) footprint_right_ = footprint[i].y;
    }

    pthread_mutex_unlock(&m_mutex);
  
  } else {
    ROS_WARN("Cannot reach service /get_footprint");
  }

}

// sets corrected velocity of joystick command
void CollisionVelocityFilter::performControllerStep() {

  double dt;
  double vx_max, vy_max;
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear = robot_twist_linear_;
  cmd_vel.angular = robot_twist_angular_;
  dt = ros::Time::now().toSec() - last_time_;
  last_time_ = ros::Time::now().toSec();

  // if closest obstacle is within stop_threshold, then do not move
  if( closest_obstacle_dist_ < stop_threshold_ ) {
    stopMovement();
    return;
  }

  double vel_angle = atan2(cmd_vel.linear.y,cmd_vel.linear.x);
  vx_max = v_max_ * fabs(cos(vel_angle));
  if (vx_max > fabs(cmd_vel.linear.x)) vx_max = fabs(cmd_vel.linear.x);
  vy_max = v_max_ * fabs(sin(vel_angle));
  if (vy_max > fabs(cmd_vel.linear.y)) vy_max = fabs(cmd_vel.linear.y);

  //Slow down in any way while approximating an obstacle:
  if(closest_obstacle_dist_ < influence_radius_) {
    double F_x, F_y;
    double vx_d, vy_d, vx_factor, vy_factor;
    double kv_obst=kv_, vx_max_obst=vx_max, vy_max_obst=vy_max;

    //implementation for linear decrease of v_max:
    double obstacle_linear_slope_x = vx_max / (obstacle_damping_dist_ - stop_threshold_);
    vx_max_obst = (closest_obstacle_dist_- stop_threshold_ + stop_threshold_/10.0f) * obstacle_linear_slope_x;
    if(vx_max_obst > vx_max) vx_max_obst = vx_max;
    else if(vx_max_obst < 0.0f) vx_max_obst = 0.0f;

    double obstacle_linear_slope_y = vy_max / (obstacle_damping_dist_ - stop_threshold_);
    vy_max_obst = (closest_obstacle_dist_- stop_threshold_ + stop_threshold_/10.0f) * obstacle_linear_slope_y;
    if(vy_max_obst > vy_max) vy_max_obst = vy_max;
    else if(vy_max_obst < 0.0f) vy_max_obst = 0.0f;

    //Translational movement
    //calculation of v factor to limit maxspeed
    double closest_obstacle_dist_x = closest_obstacle_dist_ * cos(closest_obstacle_angle_);
    double closest_obstacle_dist_y = closest_obstacle_dist_ * sin(closest_obstacle_angle_);
    vx_d = kp_/kv_obst * closest_obstacle_dist_x;
    vy_d = kp_/kv_obst * closest_obstacle_dist_y;
    vx_factor = vx_max_obst / sqrt(vy_d*vy_d + vx_d*vx_d);
    vy_factor = vy_max_obst / sqrt(vy_d*vy_d + vx_d*vx_d);
    if(vx_factor > 1.0) vx_factor = 1.0;
    if(vy_factor > 1.0) vy_factor = 1.0;

    F_x = - kv_obst * vx_last_ + vx_factor * kp_ * closest_obstacle_dist_x;
    F_y = - kv_obst * vy_last_ + vy_factor * kp_ * closest_obstacle_dist_y;

    cmd_vel.linear.x = vx_last_ + F_x / virt_mass_ * dt;
    cmd_vel.linear.y = vy_last_ + F_y / virt_mass_ * dt;

  }

  // make sure, the computed and commanded velocities are not greater than the specified max velocities
  if (fabs(cmd_vel.linear.x) > vx_max) cmd_vel.linear.x = sign(cmd_vel.linear.x) * vx_max;
  if (fabs(cmd_vel.linear.y) > vy_max) cmd_vel.linear.y = sign(cmd_vel.linear.y) * vy_max;
  if (fabs(cmd_vel.angular.z) > vtheta_max_) cmd_vel.angular.z = sign(cmd_vel.angular.z) * vtheta_max_;

  // limit acceleration:
  // only acceleration (in terms of speeding up in any direction) is limited,
  // deceleration (in terms of slowing down) is handeled either by cob_teleop or the potential field
  // like slow-down behaviour above
  if (fabs(cmd_vel.linear.x) > fabs(vx_last_))
  {
    if ((cmd_vel.linear.x - vx_last_)/dt > ax_max_)
      cmd_vel.linear.x = vx_last_ + ax_max_ * dt;
    else if((cmd_vel.linear.x - vx_last_)/dt < -ax_max_)
      cmd_vel.linear.x = vx_last_ - ax_max_ * dt;
  }
  if (fabs(cmd_vel.linear.y) > fabs(vy_last_))
  {
    if ((cmd_vel.linear.y - vy_last_)/dt > ay_max_)
      cmd_vel.linear.y = vy_last_ + ay_max_ * dt;
    else if ((cmd_vel.linear.y - vy_last_)/dt < -ay_max_)
      cmd_vel.linear.y = vy_last_ - ay_max_ * dt;
  }
  if (fabs(cmd_vel.angular.z) > fabs(vtheta_last_))
  {
    if ((cmd_vel.angular.z - vtheta_last_)/dt > atheta_max_)
      cmd_vel.angular.z = vtheta_last_ + atheta_max_ * dt;
    else if ((cmd_vel.angular.z - vtheta_last_)/dt < -atheta_max_)
      cmd_vel.angular.z = vtheta_last_ - atheta_max_ * dt;
  }

  pthread_mutex_lock(&m_mutex);
  vx_last_ = cmd_vel.linear.x;
  vy_last_ = cmd_vel.linear.y;
  vtheta_last_ = cmd_vel.angular.z;
  pthread_mutex_unlock(&m_mutex);

  // publish adjusted velocity 
  topic_pub_command_.publish(cmd_vel);  
  return;
}

void CollisionVelocityFilter::obstacleHandler() {
  pthread_mutex_lock(&m_mutex);
  if(!costmap_received_) {
    ROS_WARN("No costmap has been received by cob_collision_velocity_filter, the robot will drive without obstacle avoidance!");
    closest_obstacle_dist_ = influence_radius_;

    pthread_mutex_unlock(&m_mutex);
    return;
  }
  closest_obstacle_dist_ = influence_radius_;
  pthread_mutex_unlock(&m_mutex);

  double cur_distance_to_center, cur_distance_to_border;
  double obstacle_theta_robot, obstacle_delta_theta_robot, obstacle_dist_vel_dir;
  bool cur_obstacle_relevant;
  geometry_msgs::Point cur_obstacle_robot;
  geometry_msgs::Point zero_position;
  zero_position.x=0.0f;  
  zero_position.y=0.0f;
  zero_position.z=0.0f;
  bool use_circumscribed=true, use_tube=true;

  //Calculate corner angles in robot_frame:
  double corner_front_left, corner_rear_left, corner_rear_right, corner_front_right;
  corner_front_left = atan2(footprint_left_, footprint_front_);
  corner_rear_left = atan2(footprint_left_, footprint_rear_);
  corner_rear_right = atan2(footprint_right_, footprint_rear_);
  corner_front_right = atan2(footprint_right_, footprint_front_);

  //Decide, whether circumscribed or tube argument should be used for filtering:
  if(fabs(robot_twist_linear_.x) <= 0.005f && fabs(robot_twist_linear_.y) <= 0.005f) {
    use_tube = false;
    //disable tube filter at very slow velocities
  }
  if(!use_tube) {
    if( fabs(robot_twist_angular_.z) <= 0.01f) {
      use_circumscribed = false;
    } //when tube filter inactive, start circumscribed filter at very low rot-velocities
  } else {
    if( fabs(robot_twist_angular_.z) <= use_circumscribed_threshold_) {
      use_circumscribed = false;
    } //when tube filter running, disable circum-filter in a wider range of rot-velocities
  } 

  //Calculation of tube in driving-dir considered for obstacle avoidence
  double velocity_angle=0.0f, velocity_ortho_angle;
  double corner_angle, delta_corner_angle;
  double ortho_corner_dist;
  double tube_left_border = 0.0f, tube_right_border = 0.0f;
  double tube_left_origin = 0.0f, tube_right_origin = 0.0f;
  double corner_dist, circumscribed_radius = 0.0f;

  for(unsigned i = 0; i<robot_footprint_.size(); i++) {
    corner_dist = sqrt(robot_footprint_[i].x*robot_footprint_[i].x + robot_footprint_[i].y*robot_footprint_[i].y);
    if(corner_dist > circumscribed_radius) circumscribed_radius = corner_dist;
  }

  if(use_tube) {
    //use commanded vel-value for vel-vector direction.. ?
    velocity_angle = atan2(robot_twist_linear_.y, robot_twist_linear_.x);
    velocity_ortho_angle = velocity_angle + M_PI / 2.0f;

    for(unsigned i = 0; i<robot_footprint_.size(); i++) {
      corner_angle = atan2(robot_footprint_[i].y, robot_footprint_[i].x);
      delta_corner_angle = velocity_ortho_angle - corner_angle;
      corner_dist = sqrt(robot_footprint_[i].x*robot_footprint_[i].x + robot_footprint_[i].y*robot_footprint_[i].y);
      ortho_corner_dist = cos(delta_corner_angle) * corner_dist;

      if(ortho_corner_dist < tube_right_border) {
        tube_right_border = ortho_corner_dist;
        tube_right_origin = sin(delta_corner_angle) * corner_dist;
      } else if(ortho_corner_dist > tube_left_border) {
        tube_left_border = ortho_corner_dist;
        tube_left_origin = sin(delta_corner_angle) * corner_dist;
      }
    }
  }

  //find relevant obstacles
  pthread_mutex_lock(&m_mutex);
  relevant_obstacles_.header = last_costmap_received_.header;
  relevant_obstacles_.cell_width = last_costmap_received_.cell_width;
  relevant_obstacles_.cell_height = last_costmap_received_.cell_height;
  relevant_obstacles_.cells.clear();

  for(unsigned int i = 0; i < last_costmap_received_.cells.size(); i++) {
    cur_obstacle_relevant = false;
    cur_distance_to_center = getDistance2d(zero_position, last_costmap_received_.cells[i]);
    //check whether current obstacle lies inside the circumscribed_radius of the robot -> prevent collisions while rotating
    if(use_circumscribed && cur_distance_to_center <= circumscribed_radius) {
      cur_obstacle_robot = last_costmap_received_.cells[i];

      if( obstacleValid(cur_obstacle_robot.x, cur_obstacle_robot.y) ) {
        cur_obstacle_relevant = true;
        relevant_obstacles_.cells.push_back(last_costmap_received_.cells[i]);
        obstacle_theta_robot = atan2(cur_obstacle_robot.y, cur_obstacle_robot.x);
      }

      //for each obstacle, now check whether it lies in the tube or not:
    } else if(use_tube && cur_distance_to_center < influence_radius_) {
      cur_obstacle_robot = last_costmap_received_.cells[i];

      if( obstacleValid(cur_obstacle_robot.x, cur_obstacle_robot.y) ) {
        obstacle_theta_robot = atan2(cur_obstacle_robot.y, cur_obstacle_robot.x);
        obstacle_delta_theta_robot = obstacle_theta_robot - velocity_angle;
        obstacle_dist_vel_dir = sin(obstacle_delta_theta_robot) * cur_distance_to_center;

        if(obstacle_dist_vel_dir <= tube_left_border && obstacle_dist_vel_dir >= tube_right_border) {
          //found obstacle that lies inside of observation tube

          if( sign(obstacle_dist_vel_dir) >= 0) { 
            if(cos(obstacle_delta_theta_robot) * cur_distance_to_center >= tube_left_origin) {
              //relevant obstacle in tube found
              cur_obstacle_relevant = true;
              relevant_obstacles_.cells.push_back(last_costmap_received_.cells[i]);
            }
          } else { // obstacle in right part of tube
            if(cos(obstacle_delta_theta_robot) * cur_distance_to_center >= tube_right_origin) {
              //relevant obstacle in tube found
              cur_obstacle_relevant = true;
              relevant_obstacles_.cells.push_back(last_costmap_received_.cells[i]);
            }
          }
        }
      }
    }
    
    if(cur_obstacle_relevant) { //now calculate distance of current, relevant obstacle to robot
      if(obstacle_theta_robot >= corner_front_right && obstacle_theta_robot < corner_front_left) {
        //obstacle in front:
        cur_distance_to_border = cur_distance_to_center - fabs(footprint_front_) / fabs(cos(obstacle_theta_robot));
      } else if(obstacle_theta_robot >= corner_front_left && obstacle_theta_robot < corner_rear_left) {
        //obstacle left:
        cur_distance_to_border = cur_distance_to_center - fabs(footprint_left_) / fabs(sin(obstacle_theta_robot));
      } else if(obstacle_theta_robot >= corner_rear_left || obstacle_theta_robot < corner_rear_right) {
        //obstacle in rear:
        cur_distance_to_border = cur_distance_to_center - fabs(footprint_rear_) / fabs(cos(obstacle_theta_robot));
      } else {
        //obstacle right:
        cur_distance_to_border = cur_distance_to_center - fabs(footprint_right_) / fabs(sin(obstacle_theta_robot));
      }

      if(cur_distance_to_border < closest_obstacle_dist_) {
        closest_obstacle_dist_ = cur_distance_to_border;
        closest_obstacle_angle_ = obstacle_theta_robot;
      }
    }	
  }
  pthread_mutex_unlock(&m_mutex);

  topic_pub_relevant_obstacles_.publish(relevant_obstacles_);
}

// load robot footprint from costmap_2d_ros to keep same footprint format
std::vector<geometry_msgs::Point> CollisionVelocityFilter::loadRobotFootprint(ros::NodeHandle node){
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
  ROS_DEBUG("Extracted rectangular footprint for cob_collision_velocity_filter: Front: %f, Rear %f, Left: %f, Right %f", footprint_front_, footprint_rear_, footprint_left_, footprint_right_);

  return footprint;
}

double CollisionVelocityFilter::getDistance2d(geometry_msgs::Point a, geometry_msgs::Point b) {
  return sqrt( pow(a.x - b.x,2) + pow(a.y - b.y,2) );
}

double CollisionVelocityFilter::sign(double x) {
  if(x >= 0.0f) return 1.0f;
  else return -1.0f;
}

bool CollisionVelocityFilter::obstacleValid(double x_obstacle, double y_obstacle) {
  if(x_obstacle<footprint_front_ && x_obstacle>footprint_rear_ && y_obstacle>footprint_right_ && y_obstacle<footprint_left_) {
    ROS_WARN("Found an obstacle inside robot_footprint: Skip!");
    return false;
  }

  return true;
}

void CollisionVelocityFilter::stopMovement() {
  geometry_msgs::Twist stop_twist;
  stop_twist.linear.x = 0.0f; stop_twist.linear.y = 0.0f; stop_twist.linear.z = 0.0f;
  stop_twist.angular.x = 0.0f; stop_twist.angular.y = 0.0f; stop_twist.linear.z = 0.0f;
  topic_pub_command_.publish(stop_twist);
  vx_last_ = 0.0;
  vy_last_ = 0.0;
  vtheta_last_ = 0.0;
}

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
  // initialize ROS, spezify name of node
  ros::init(argc, argv, "cob_collision_velocity_filter");

  // create nodeClass
  CollisionVelocityFilter collisionVelocityFilter;


  ros::spin();

  return 0;
}

