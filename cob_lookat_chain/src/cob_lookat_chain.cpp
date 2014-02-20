/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2014 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: cob_lookat_chain
 * \note
 *   ROS stack name: cob_driver
 * \note
 *   ROS package name: cob_lookat_chain
 *
 * \author
 *   Author: Felix Messmer, email:felix.messmer@ipa.fhg.de
 *
 * \date Date of creation: Feb 2014
 *
 * \brief
 *   Implementation of ROS node for lookat_chain.
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
//##################

// standard includes
// --

// ROS includes
#include <ros/ros.h>
#include <urdf/model.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>


// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>

// own includes
#include <cob_lookat_chain/LookatParams.h>
#include <cob_lookat_chain/LookatCtrl.h>

/*!
 * \brief Implementation of ROS node for lookat_chain.
 *
 * Offers velocity and position interface.
 */
class LookatChainNode
{

public:
  /// create a handle for this node, initialize node
  ros::NodeHandle n_;

  /// declaration of topics to publish
  ros::Publisher topicPub_JointState_;
  ros::Publisher topicPub_ControllerState_;
  ros::Publisher topicPub_OperationMode_;
  ros::Publisher topicPub_Diagnostic_;

  /// declaration of topics to subscribe, callback is called for new messages arriving
  ros::Subscriber topicSub_CommandPos_;
  ros::Subscriber topicSub_CommandVel_;

  /// declaration of service servers
  ros::ServiceServer srvServer_Init_;
  ros::ServiceServer srvServer_SetOperationMode_;
  ros::ServiceServer srvServer_Stop_;
  ros::ServiceServer srvServer_Recover_;

  /// handle for lookat_chain parameters
  LookatParams* params_;
  LookatCtrl* lookat_ctrl_;

  /// member variables
  bool initialized_;
  bool stopped_;
  bool error_;
  std::string error_msg_;
  ros::Time last_publish_time_;

  ///Constructor
  LookatChainNode()
  {
    n_ = ros::NodeHandle("~");
    
    unsigned int DOF = 4;
    params_ = new LookatParams(DOF);
    lookat_ctrl_ = new LookatCtrl(params_);

    /// implementation of topics to publish
    topicPub_JointState_ = n_.advertise<sensor_msgs::JointState> ("/joint_states", 1);
    topicPub_ControllerState_ = n_.advertise<control_msgs::JointTrajectoryControllerState> ("state", 1);
    topicPub_OperationMode_ = n_.advertise<std_msgs::String> ("current_operationmode", 1);
    topicPub_Diagnostic_ = n_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

    /// implementation of topics to subscribe
    topicSub_CommandPos_ = n_.subscribe("command_pos", 1, &LookatChainNode::topicCallback_CommandPos, this);
    topicSub_CommandVel_ = n_.subscribe("command_vel", 1, &LookatChainNode::topicCallback_CommandVel, this);

    /// implementation of service servers
    srvServer_Init_ = n_.advertiseService("init", &LookatChainNode::srvCallback_Init, this);
    srvServer_Stop_ = n_.advertiseService("stop", &LookatChainNode::srvCallback_Stop, this);
    srvServer_Recover_ = n_.advertiseService("recover", &LookatChainNode::srvCallback_Recover, this);
    srvServer_SetOperationMode_ = n_.advertiseService("set_operation_mode", &LookatChainNode::srvCallback_SetOperationMode, this);

    initialized_ = true;    //lookat_chain does not need to be initialized
    stopped_ = true;
    error_ = false;
    last_publish_time_ = ros::Time::now();
  }

  /// Destructor
  ~LookatChainNode()
  {
    ROS_INFO("LookatChainNode closed!");
  }

  /*!
   * \brief Gets parameters from the ROS parameter server and configures the lookat_chain.
   */
  void getROSParameters()
  {
    /// get force_use_movevel
    bool UseMoveVel;
    if (n_.hasParam("force_use_movevel"))
    {
      n_.getParam("force_use_movevel", UseMoveVel);
      ROS_INFO("Parameter force_use_movevel set, using moveVel");
    }
    else
    {
      ROS_INFO("Parameter force_use_movevel not set, using moveStep");
      UseMoveVel = false;
    }
    params_->SetUseMoveVel(UseMoveVel);

    /// Get joint names
    XmlRpc::XmlRpcValue JointNamesXmlRpc;
    std::vector<std::string> JointNames;
    if (n_.hasParam("joint_names"))
    {
      n_.getParam("joint_names", JointNamesXmlRpc);
    }
    else
    {
      ROS_ERROR("Parameter joint_names not set, shutting down node...");
      n_.shutdown();
    }

    /// Resize and assign of values to the JointNames
    JointNames.resize(JointNamesXmlRpc.size());
    for (int i = 0; i < JointNamesXmlRpc.size(); i++)
    {
      JointNames[i] = (std::string)JointNamesXmlRpc[i];
    }

    /// Check dimension with with DOF
    if (JointNames.size() != params_->GetDOF())
    {
      ROS_ERROR("Wrong dimensions of parameter joint_names, shutting down node...");
      n_.shutdown();
    }
    params_->SetJointNames(JointNames);

    /// Get max accelerations
    XmlRpc::XmlRpcValue MaxAccelerationsXmlRpc;
    std::vector<double> MaxAccelerations;
    if (n_.hasParam("max_accelerations"))
    {
      n_.getParam("max_accelerations", MaxAccelerationsXmlRpc);
    }
    else
    {
      ROS_ERROR("Parameter max_accelerations not set, shutting down node...");
      n_.shutdown();
    }

    /// Resize and assign of values to the MaxAccelerations
    MaxAccelerations.resize(MaxAccelerationsXmlRpc.size());
    for (int i = 0; i < MaxAccelerationsXmlRpc.size(); i++)
    {
      MaxAccelerations[i] = (double)MaxAccelerationsXmlRpc[i];
    }

    /// Check dimension with with DOF
    if (MaxAccelerations.size() != params_->GetDOF())
    {
      ROS_ERROR("Wrong dimensions of parameter max_accelerations, shutting down node...");
      n_.shutdown();
    }
    params_->SetMaxAcc(MaxAccelerations);
  }

  /*!
   * \brief Gets parameters from the robot_description and configures the lookat_chain.
   */
  void getRobotDescriptionParameters()
  {
    unsigned int DOF = params_->GetDOF();
    std::vector<std::string> JointNames = params_->GetJointNames();

    /// Get robot_description from ROS parameter server
    std::string param_name = "robot_description";
    std::string full_param_name;
    std::string xml_string;
    n_.searchParam(param_name, full_param_name);
    if (n_.hasParam(full_param_name))
    {
      n_.getParam(full_param_name.c_str(), xml_string);
    }
    else
    {
      ROS_ERROR("Parameter %s not set, shutting down node...", full_param_name.c_str());
      n_.shutdown();
    }

    if (xml_string.size() == 0)
    {
      ROS_ERROR("Unable to load robot model from parameter %s",full_param_name.c_str());
      n_.shutdown();
    }
    ROS_DEBUG("%s content\n%s", full_param_name.c_str(), xml_string.c_str());

    /// Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
      ROS_ERROR("Failed to parse urdf file");
      n_.shutdown();
    }
    ROS_DEBUG("Successfully parsed urdf file");

    /// Get max velocities out of urdf model
    std::vector<double> MaxVelocities(DOF);
    for (unsigned int i = 0; i < DOF; i++)
    {
      MaxVelocities[i] = model.getJoint(JointNames[i].c_str())->limits->velocity;
    }

    /// Get lower limits out of urdf model
    std::vector<double> LowerLimits(DOF);
    for (unsigned int i = 0; i < DOF; i++)
    {
      LowerLimits[i] = model.getJoint(JointNames[i].c_str())->limits->lower;
    }

    // Get upper limits out of urdf model
    std::vector<double> UpperLimits(DOF);
    for (unsigned int i = 0; i < DOF; i++)
    {
      UpperLimits[i] = model.getJoint(JointNames[i].c_str())->limits->upper;
    }

    /// Set parameters
    params_->SetMaxVel(MaxVelocities);
    params_->SetLowerLimits(LowerLimits);
    params_->SetUpperLimits(UpperLimits);
    
    lookat_ctrl_->Init(params_);
  }

  /*!
   * \brief Executes the callback from the command_pos topic.
   *
   * Set the current position target.
   * \param msg JointPositions
   */
  void topicCallback_CommandPos(const brics_actuator::JointPositions::ConstPtr& msg)
  {
    ROS_WARN("Received new position command. Skipping command: Position commands currently not implemented");
  }

  /*!
   * \brief Executes the callback from the command_vel topic.
   *
   * Set the current velocity target.
   * \param msg JointVelocities
   */
  void topicCallback_CommandVel(const brics_actuator::JointVelocities::ConstPtr& msg)
  {
    ROS_DEBUG("Received new velocity command");

    /// ToDo: don't rely on position of joint names, but merge them (check between msg.joint_uri and member variable JointStates)
    
    unsigned int DOF = params_->GetDOF();
    std::vector<std::string> jointNames = params_->GetJointNames();
    std::vector<double> cmd_vel(DOF);
    std::string unit = "rad";

    /// check dimensions
    if (msg->velocities.size() != DOF)
    {
      ROS_ERROR("Skipping command: Commanded velocities and DOF are not same dimension.");
      return;
    }

    /// parse velocities
    for (unsigned int i = 0; i < DOF; i++)
    {
      /// check joint name
      if (msg->velocities[i].joint_uri != jointNames[i])
      {
        ROS_ERROR("Skipping command: Received joint name %s doesn't match expected joint name %s for joint %d.",msg->velocities[i].joint_uri.c_str(),jointNames[i].c_str(),i);
        return;
      }

      /// check unit
      if (msg->velocities[i].unit != unit)
      {
        ROS_ERROR("Skipping command: Received unit %s doesn't match expected unit %s.",msg->velocities[i].unit.c_str(),unit.c_str());
        return;
      }

      /// if all checks are successful, parse the velocity value for this joint
      ROS_DEBUG("Parsing velocity %f for joint %s",msg->velocities[i].value,jointNames[i].c_str());
      cmd_vel[i] = msg->velocities[i].value;
    }
    
    /// command velocities to lookat_chain
    if (!lookat_ctrl_->MoveVel(cmd_vel))
    {
      error_ = true;
      error_msg_ = "Something went wrong during execution";
      ROS_ERROR("Skipping command: An Error occured");
      return;
    }

    ROS_DEBUG("Executed velocity command");
   
    publishState(false);
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
    ROS_INFO("Initializing lookat_chain...");
    
    lookat_ctrl_->Init(params_);
    
    initialized_ = true;
    res.success.data = true;
    ROS_INFO("...initializing lookat_chain successful");

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
    ROS_INFO("Stopping lookat_chain...");

    /// stop lookat_chain
    res.success.data = true;
    ROS_INFO("...stopping lookat_chain successful.");

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
    ROS_INFO("Recovering lookat_chain...");
    
    error_ = false;
    error_msg_ = "";
    res.success.data = true;
    ROS_INFO("...recovering lookat_chain successful.");

    return true;
  }

   /*!
   * \brief Executes the service callback for SetOperationMode.
   *
   * Sets the driver to different operation modes. Currently only operation_mode=velocity is supported.
   * \param req Service request
   * \param res Service response
   */
  bool srvCallback_SetOperationMode(cob_srvs::SetOperationMode::Request &req, cob_srvs::SetOperationMode::Response &res)
  {
    if(req.operation_mode.data != "velocity")
    {
      ROS_WARN("lookat_chain currently only supports velocity commands");
      res.success.data = false;
    }
    else
    {
      res.success.data = true;
    }
    return true;
  }

  /*!
   * \brief Publishes the state of the lookat_chain as ros messages.
   *
   * Published to "/joint_states" as "sensor_msgs/JointState"
   * Published to "state" as "control_msgs/JointTrajectoryControllerState"
   */
  void publishState(bool update=true)
  {
    if (initialized_)
    {
      ROS_DEBUG("publish state");

      if(update)
      {lookat_ctrl_->updateStates();}

      sensor_msgs::JointState joint_state_msg;
      joint_state_msg.header.stamp = ros::Time::now();
      joint_state_msg.name = params_->GetJointNames();
      joint_state_msg.position = lookat_ctrl_->getPositions();
      joint_state_msg.velocity = lookat_ctrl_->getVelocities();
      joint_state_msg.effort.resize(params_->GetDOF());

      control_msgs::JointTrajectoryControllerState controller_state_msg;
      controller_state_msg.header.stamp = joint_state_msg.header.stamp;
      controller_state_msg.joint_names = params_->GetJointNames();
      controller_state_msg.actual.positions = lookat_ctrl_->getPositions();
      controller_state_msg.actual.velocities = lookat_ctrl_->getVelocities();
      controller_state_msg.actual.accelerations = lookat_ctrl_->getAccelerations();

      std_msgs::String opmode_msg;
      opmode_msg.data = "velocity";

      /// publishing joint and controller states on topic
      topicPub_JointState_.publish(joint_state_msg);
      topicPub_ControllerState_.publish(controller_state_msg);
      topicPub_OperationMode_.publish(opmode_msg);

      last_publish_time_ = joint_state_msg.header.stamp;
    }


    // publishing diagnotic messages
    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostics.status.resize(1);

    // set data to diagnostics
    if(error_)
    {
      diagnostics.status[0].level = 2;
      diagnostics.status[0].name = n_.getNamespace();;
      diagnostics.status[0].message =  "LookatChain: An Error occured";
    }
    else
    {
      if (initialized_)
      {
        diagnostics.status[0].level = 0;
        diagnostics.status[0].name = n_.getNamespace(); //"lookat_chain";
        diagnostics.status[0].message = "lookat_chain initialized and running";
      }
      else
      {
        diagnostics.status[0].level = 1;
        diagnostics.status[0].name = n_.getNamespace(); //"lookat_chain";
        diagnostics.status[0].message = "lookat_chain not initialized";
      }
    }
    // publish diagnostic message
    topicPub_Diagnostic_.publish(diagnostics);
  }
}; //LookatChainNode

/*!
 * \brief Main loop of ROS node.
 *
 * Running with a specific frequency defined by loop_rate.
 */
int main(int argc, char** argv)
{
  /// initialize ROS, specify name of node
  ros::init(argc, argv, "lookat_chain");

  // create LookatChainNode
  LookatChainNode node;
  node.getROSParameters();
  node.getRobotDescriptionParameters();

  /// get main loop parameters
  double frequency;
  if (node.n_.hasParam("frequency"))
  {
    node.n_.getParam("frequency", frequency);
    //frequency of driver has to be much higher then controller frequency
  }
  else
  {
    //frequency of driver has to be much higher then controller frequency
    frequency = 10; //Hz
    ROS_WARN("Parameter frequency not available, setting to default value: %f Hz", frequency);
  }

  ros::Duration min_publish_duration;
  if (node.n_.hasParam("min_publish_duration"))
  {
    double sec;
    node.n_.getParam("min_publish_duration", sec);
    min_publish_duration.fromSec(sec);
  }
  else
  {
    ROS_ERROR("Parameter min_publish_time not available");
    return 0;
  }

  if((1.0/min_publish_duration.toSec()) > frequency)
  {
    ROS_ERROR("min_publish_duration has to be longer then delta_t of controller frequency!");
    return 0;
  }

  /// main loop
  ros::Rate loop_rate(frequency); // Hz
  while (node.n_.ok())
  {
    if ((ros::Time::now() - node.last_publish_time_) >= min_publish_duration)
    {
      node.publishState();
    }

    /// sleep and waiting for messages, callbacks
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
