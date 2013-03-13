/****************************************************************
 *
 * Copyright (c) 2013
 *
 * Fraunhofer Institute for Manufacturing Engineering  
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_base_velocity_smoother
 *  							
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  		
 * Author: Florian Mirus, email:Florian.Mirus@ipa.fhg.de
 *
 * Date of creation: Jan 2013
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

#ifndef COB_BASE_VELOCITY_SMOOTHER_H
#define COB_BASE_VELOCITY_SMOOTHER_H

//**************************** includes ************************/

// standard includes
#include <XmlRpc.h>
#include <pthread.h>
#include <deque>
#include <sstream>
#include <iostream>
#include <boost/circular_buffer.hpp>
#include <boost/bind.hpp>

using namespace std;

// ros includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <std_msgs/String.h>
/****************************************************************
 * the ros navigation doesn't run very smoothly because acceleration is too high
 * --> cob has strong base motors and therefore reacts with shaking behavior 
 * (PR2 has much more mechanical damping)
 * solution: the additional node cob_base_velocity_smoother smooths the velocities
 * comming from ROS-navigation or teleoperation, by calculating the mean values of a certain number 
 * of past messages and limiting the acceleration under a given threshold. 
 * cob_base_velocity_smoother subsribes (input) and publishes (output) geometry_msgs::Twist.
 ****************************************************************/
class cob_base_velocity_smoother
{
private:
  //capacity for circular buffers (to be loaded from parameter server, otherwise set to default value 12)
  int buffer_capacity_;
  //maximal time-delay in seconds for stored messages in Circular Buffer (to be loaded from parameter server, otherwise set to default value 4)
  double store_delay_;
  // maximal time-delay in seconds to wait until filling the circular buffer with zero messages when the subscriber doesn't hear anything
  double stop_delay_after_no_sub_;
  //threshhold for maximal allowed acceleration (to be loaded from parameter server, otherwise set to default value 0.02)
  double acc_limit_;
  // ros loop rate (to be loaded from parameter server, otherwise set to default value 30)
  double loop_rate_;
  //geometry message filled with zero values
  geometry_msgs::Twist zero_values_;
  // subscribed geometry message
  geometry_msgs::Twist sub_msg_;
  // first time no incoming messages
  ros::Time first_time_no_sub_;
  // bool to check if it is the first time for first_time_no_sub_ to be set
  bool no_sub_time_set_;

public:

  // constructor
  cob_base_velocity_smoother();

  // destructor
  ~cob_base_velocity_smoother();

  //create node handle
  ros::NodeHandle nh_, pnh_;

  //circular buffers for velocity, output and time
  boost::circular_buffer<geometry_msgs::Twist> cb_;
  boost::circular_buffer<geometry_msgs::Twist> cb_out_;
  boost::circular_buffer<ros::Time> cb_time_;

  // declaration of ros subscribers
  ros::Subscriber geometry_msgs_sub_;

  // decalaration of ros publishers
  ros::Publisher pub_;

  //callback function to subsribe to the geometry messages
  void geometryCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel);
  //calculation function called periodically in main
  void calculationStep();
  //function that updates the circular buffer after receiving a new geometry message
  void reviseCircBuff(ros::Time now, geometry_msgs::Twist cmd_vel);
  //function to limit the acceleration under the given threshhold thresh
  void limitAcceleration(ros::Time now, geometry_msgs::Twist& cmd_vel);

  //boolean function that returns true if all messages stored in the circular buffer are older than store_delay, false otherwise
  bool circBuffOutOfDate(ros::Time now);
  // function to compare two geometry messages
  bool IsEqual(geometry_msgs::Twist msg1, geometry_msgs::Twist msg2);

  //boolean function that returns true if the input msg cmd_vel equals zero_values, false otherwise
  bool IsZeroMsg(geometry_msgs::Twist cmd_vel);

  //help-function that returns the signum of a double variable
  int signum(double var);

  //functions to calculate the mean values for each direction
  double meanValueX();
  double meanValueY();
  double meanValueZ();

  // function to make the loop rate available outside the class
  double getLoopRate();

  //function for the actual computation
  //calls the reviseCircBuff and the meanValue-functions and limits the acceleration under thresh
  //returns the resulting geometry message to be published to the base_controller
  geometry_msgs::Twist setOutput(ros::Time now, geometry_msgs::Twist cmd_vel);

};

#endif
