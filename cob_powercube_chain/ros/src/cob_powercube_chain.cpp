/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2010 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_driver
 * \note
 *   ROS package name: cob_powercube_chain
 *
 * \author
 *   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * \author
 *   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 *
 * \date Date of creation: Jan 2010
 *
 * \brief
 *   Implementation of ROS node for powercube_chain.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
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

//##################
//#### includes ####

// standard includes
//

// ROS includes
#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>

// ROS service includes
#include <cob_srvs/Trigger.h>

// own includes
#include <cob_powercube_chain/PowerCubeCtrl.h>
#include <cob_powercube_chain/PowerCubeCtrlParams.h>

/*!
 * \brief Implementation of ROS node for powercube_chain.
 *
 * Offers velocity and position interface.
 */

class PowerCubeChainNode
{

public:
  /// create a handle for this node, initialize node
  ros::NodeHandle n_;

  // declaration of topics to publish
  ros::Publisher topicPub_JointState_;
  ros::Publisher topicPub_ControllerState_;

  // declaration of topics to subscribe, callback is called for new messages arriving
  ros::Subscriber topicSub_CommandPos_;
  ros::Subscriber topicSub_CommandVel_;

  // declaration of service servers
  ros::ServiceServer srvServer_Init_;
  ros::ServiceServer srvServer_Stop_;
  ros::ServiceServer srvServer_Recover_;

  /// Handle for powercube_chain
  PowerCubeCtrl* pc_ctrl_;

  /// Handle for powercube_chain parameters
  PowerCubeCtrlParams* pc_params_;

  // member variables
  bool init_request_;
  bool initialized_;
  bool stop_request_;
  bool stopped_;

  PowerCubeChainNode()
  {
    pc_ctrl_ = new PowerCubeCtrl();
    pc_params_ = new PowerCubeCtrlParams();

    // implementation of topics to publish
    topicPub_JointState_ = n_.advertise<sensor_msgs::JointState> ("/joint_states", 1);
    topicPub_ControllerState_ = n_.advertise<pr2_controllers_msgs::JointTrajectoryControllerState> ("state", 1);

    // implementation of topics to subscribe
    topicSub_CommandPos_ = n_.subscribe("command_pos", 1, &PowerCubeChainNode::topicCallback_CommandPos, this);
    topicSub_CommandVel_ = n_.subscribe("command_vel", 1, &PowerCubeChainNode::topicCallback_CommandVel, this);

    // implementation of service servers
    srvServer_Init_ = n_.advertiseService("init", &PowerCubeChainNode::srvCallback_Init, this);
    srvServer_Stop_ = n_.advertiseService("stop", &PowerCubeChainNode::srvCallback_Stop, this);
    srvServer_Recover_ = n_.advertiseService("recover", &PowerCubeChainNode::srvCallback_Recover, this);

    initialized_ = false;
  }

  ~PowerCubeChainNode()
  {
    bool closed = pc_ctrl_->Close();
    if (closed)
      ROS_INFO("PowerCube Device closed!");
  }

  void initParameters()
  {
    std::string CanModule;
    int CanDevice;
    int CanBaudrate;
    XmlRpc::XmlRpcValue ModIdsXmlRpc;
    std::vector<int> ModIds;
    XmlRpc::XmlRpcValue JointNamesXmlRpc;
    std::vector<std::string> JointNames;

    // initialize parameters
    pc_params_->Init(CanModule, CanDevice, CanBaudrate, ModIds);
  }

  /*!
   * \brief Executes the callback from the command_pos topic.
   *
   * Set the current position target.
   * \param msg JointPositions
   */
  void topicCallback_CommandPos(const brics_actuator::JointPositions::ConstPtr& msg)
  {
    ROS_INFO("Received new position command");
  }

  /*!
   * \brief Executes the callback from the command_vel topic.
   *
   * Set the current velocity target.
   * \param msg JointVelocities
   */
  void topicCallback_CommandVel(const brics_actuator::JointVelocities::ConstPtr& msg)
  {
    ROS_INFO("Received new velocity command");
  }

  /*!
   * \brief Executes the service callback for init.
   *
   * Connects to the hardware and initialized it.
   * \param req Service request
   * \param res Service response
   */
  bool srvCallback_Init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
  {
    if (initialized_ == false)
    {
      ROS_INFO("...initializing powercubes...");

      // initialize powercubes
      if (pc_ctrl_->Init(pc_params_))
      {
        ROS_INFO("Initializing successful");
        initialized_ = true;
        res.success.data = true;
        res.error_message.data = "Initializing successful";
        ROS_INFO("...%s",res.error_message.data);
      }
      else
      {
        res.success.data = false;
        res.error_message.data = pc_ctrl_->getErrorMessage();
        ROS_ERROR("Initializing powercubes not successful. error: %s", res.error_message.data);
      }
    }
    else
    {
      res.success.data = false;
      res.error_message.data = "powercubes already initialized";
      ROS_ERROR("...%s", res.error_message.data);
    }

    return true;
  }

  /*!
   * \brief Executes the service callback for stop.
   *
   * Stops all hardware movements.
   * \param req Service request
   * \param res Service response
   */
  bool srvCallback_Stop(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
  {
    ROS_INFO("Stopping powercubes...");

    // stop powercubes
    if (pc_ctrl_->Stop())
    {
      res.success.data = true;
      res.error_message.data = "stopping powercubes succesfull";
      ROS_INFO("...%s",res.error_message.data);
    }
    else
    {
      res.success.data = false;
      res.error_message.data = pc_ctrl_->getErrorMessage();
      ROS_ERROR("Stopping powercubes not succesfull. error: %s", res.error_message.data);
    }
    return true;
  }

  /*!
   * \brief Executes the service callback for recover.
   *
   * Recovers the driver after an emergency stop.
   * \param req Service request
   * \param res Service response
   */
  bool srvCallback_Recover(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
  {
    if (initialized_ == true)
    {
      ROS_INFO("Recovering powercubes");

      // stopping all arm movements
      if (pc_ctrl_->Recover())
      {
        ROS_INFO("Recovering powercubes succesfull");
        res.success.data = true;
      }
      else
      {
        res.success.data = false;
        res.error_message.data = pc_ctrl_->getErrorMessage();
        ROS_ERROR("Recovering powercubes not succesfull. error: %s", res.error_message.data);
      }
    }
    else
    {
      res.success.data = false;
      res.error_message.data = "powercubes not initialized";
      ROS_ERROR("...%s...",res.error_message.data);
    }

    return true;
  }

}; //PowerCubeChainNode

/*!
 * \brief Main loop of ROS node.
 *
 * Running with a specific frequency defined by loop_rate.
 */
int main(int argc, char** argv)
{
  // initialize ROS, specify name of node
  ros::init(argc, argv, "powercube_chain");

  // create PowerCubeChainNode
  PowerCubeChainNode pc_node;
  pc_node.initParameters();

  // main loop
  double frequency;
  if (pc_node.n_.hasParam("frequency"))
  {
    pc_node.n_.getParam("frequency", frequency);
  }
  else
  {
    frequency = 10; //Hz
    ROS_WARN("Parameter frequency not available, setting to default value: %f Hz", frequency);
  }

  ros::Rate loop_rate(frequency); // Hz
  while (pc_node.n_.ok())
  {
    ROS_INFO("main");
    // sleep and waiting for messages, callbacks
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
