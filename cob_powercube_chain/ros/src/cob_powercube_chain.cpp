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
 * \date Date of creation: Dec 2010
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
//--

// ROS includes
#include <ros/ros.h>
#include <urdf/model.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
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
  bool initialized_;
  bool stopped_;
  ros::Time last_publish_time_;

  PowerCubeChainNode()
  {
    pc_params_ = new PowerCubeCtrlParams();
    pc_ctrl_ = new PowerCubeCtrl(pc_params_);

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
    stopped_ = true;
    last_publish_time_ = ros::Time::now();
  }

  ~PowerCubeChainNode()
  {
    bool closed = pc_ctrl_->Close();
    if (closed)
      ROS_INFO("PowerCube Device closed!");
  }

  /*!
   * \brief Gets parameters from the ROS parameter server and configures the powercube_chain.
   */
  void getROSParameters()
  {
    // get CanModule
    std::string CanModule;
    if (n_.hasParam("can_module"))
    {
      n_.getParam("can_module", CanModule);
    }
    else
    {
      ROS_ERROR("Parameter can_module not set, shutting down node...");
      n_.shutdown();
    }

    // get CanDevice
    std::string CanDevice;
    if (n_.hasParam("can_device"))
    {
      n_.getParam("can_device", CanDevice);
    }
    else
    {
      ROS_ERROR("Parameter can_device not set, shutting down node...");
      n_.shutdown();
    }

    // get CanBaudrate
    int CanBaudrate;
    if (n_.hasParam("can_baudrate"))
    {
      n_.getParam("can_baudrate", CanBaudrate);
    }
    else
    {
      ROS_ERROR("Parameter can_baudrate not set, shutting down node...");
      n_.shutdown();
    }

    // get modul ids
    XmlRpc::XmlRpcValue ModulIDsXmlRpc;
    std::vector<int> ModulIDs;
    if (n_.hasParam("modul_ids"))
    {
      n_.getParam("modul_ids", ModulIDsXmlRpc);
    }
    else
    {
      ROS_ERROR("Parameter modul_ids not set, shutting down node...");
      n_.shutdown();
    }
    ModulIDs.resize(ModulIDsXmlRpc.size());
    for (int i = 0; i < ModulIDsXmlRpc.size(); i++)
    {
      ModulIDs[i] = (int)ModulIDsXmlRpc[i];
    }

    // init parameters
    pc_params_->Init(CanModule, CanDevice, CanBaudrate, ModulIDs);

    // get joint names
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
    JointNames.resize(JointNamesXmlRpc.size());
    for (int i = 0; i < JointNamesXmlRpc.size(); i++)
    {
      JointNames[i] = (std::string)JointNamesXmlRpc[i];
    }
    // check dimension with with DOF
    if ((int)JointNames.size() != pc_params_->GetDOF())
    {
      ROS_ERROR("Wrong dimensions of parameter joint_names, shutting down node...");
      n_.shutdown();
    }
    pc_params_->SetJointNames(JointNames);

    // get max accelerations
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
    MaxAccelerations.resize(MaxAccelerationsXmlRpc.size());
    for (int i = 0; i < MaxAccelerationsXmlRpc.size(); i++)
    {
      MaxAccelerations[i] = (double)MaxAccelerationsXmlRpc[i];
    }
    // check dimension with with DOF
    if ((int)MaxAccelerations.size() != pc_params_->GetDOF())
    {
      ROS_ERROR("Wrong dimensions of parameter max_accelerations, shutting down node...");
      n_.shutdown();
    }
    pc_params_->SetMaxAcc(MaxAccelerations);

    // get horizon
    double Horizon;
    if (n_.hasParam("horizon"))
    {
      n_.getParam("horizon", Horizon);
    }
    else
    {
      Horizon = 0.025; //Hz
      ROS_WARN("Parameter horizon not available, setting to default value: %f sec", Horizon);
    }
    pc_ctrl_->setHorizon(Horizon);
  }

  /*!
   * \brief Gets parameters from the robot_description and configures the powercube_chain.
   */
  void getRobotDescriptionParameters()
  {
    unsigned int DOF = pc_params_->GetDOF();
    std::vector<std::string> JointNames = pc_params_->GetJointNames();

    // get robot_description from ROS parameter server
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

    // get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
      ROS_ERROR("Failed to parse urdf file");
      n_.shutdown();
    }
    ROS_DEBUG("Successfully parsed urdf file");

    // get max velocities out of urdf model
    std::vector<double> MaxVelocities(DOF);
    for (unsigned int i = 0; i < DOF; i++)
    {
      MaxVelocities[i] = model.getJoint(JointNames[i].c_str())->limits->velocity;
    }

    // get lower limits out of urdf model
    std::vector<double> LowerLimits(DOF);
    for (unsigned int i = 0; i < DOF; i++)
    {
      LowerLimits[i] = model.getJoint(JointNames[i].c_str())->limits->lower;
    }

    // get upper limits out of urdf model
    std::vector<double> UpperLimits(DOF);
    for (unsigned int i = 0; i < DOF; i++)
    {
      UpperLimits[i] = model.getJoint(JointNames[i].c_str())->limits->upper;
    }

    // get offsets out of urdf model
    std::vector<double> Offsets(DOF);
    for (unsigned int i = 0; i < DOF; i++)
    {
      Offsets[i] = model.getJoint(JointNames[i].c_str())->calibration->rising.get()[0];
    }

    // set parameters
    pc_params_->SetMaxVel(MaxVelocities);
    pc_params_->SetLowerLimits(LowerLimits);
    pc_params_->SetUpperLimits(UpperLimits);
    pc_params_->SetOffsets(Offsets);
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
    if (initialized_)
    {

      PowerCubeCtrl::PC_CTRL_STATUS status;
      std::vector<std::string> errorMessages;
      ROS_WARN("here");
      pc_ctrl_->getStatus(status, errorMessages);
      ROS_WARN("here2");
      std::cout << status << std::endl;

      // @todo don't rely on position of joint names, but merge them (check between msg.joint_uri and member variable JointStates)

      unsigned int DOF = pc_params_->GetDOF();
      std::vector<std::string> jointNames = pc_params_->GetJointNames();
      std::vector<double> cmd_vel(DOF);
      std::string unit = "rad";

      // check dimensions
      if (msg->velocities.size() != DOF)
      {
        ROS_ERROR("Skipping command: Commanded velocities and DOF are not same dimension.");
        return;
      }

      // parse velocities
      for (unsigned int i = 0; i < DOF; i++)
      {
        // check joint name
        if (msg->velocities[i].joint_uri != jointNames[i])
        {
          ROS_ERROR("Skipping command: Received joint name %s doesn't match expected joint name %s for joint %d.",msg->velocities[i].joint_uri.c_str(),jointNames[i].c_str(),i);
          return;
        }

        // check unit
        if (msg->velocities[i].unit != unit)
        {
          ROS_ERROR("Skipping command: Received unit %s doesn't match expected unit %s.",msg->velocities[i].unit.c_str(),unit.c_str());
          return;
        }

        // if all checks are successful, parse the velocity value for this joint
        ROS_DEBUG("Parsing velocity %f for joint %s",msg->velocities[i].value,jointNames[i].c_str());
        cmd_vel[i] = msg->velocities[i].value;
      }

      // command velocities to powercubes
      if (!pc_ctrl_->MoveVel(cmd_vel))
      {
        ROS_ERROR("Skipping command: %s",pc_ctrl_->getErrorMessage().c_str());
        return;
      }

      ROS_DEBUG("Executed velocity command");
    }
    else
    {
      ROS_ERROR("Skipping command: powercubes not initialized");
    }
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
    if (!initialized_)
    {
      ROS_INFO("Initializing powercubes...");

      // initialize powercubes
      if (pc_ctrl_->Init(pc_params_))
      {
        initialized_ = true;
        res.success.data = true;
        ROS_INFO("...initializing powercubes successful");
      }
      else
      {
        res.success.data = false;
        res.error_message.data = pc_ctrl_->getErrorMessage();
        ROS_ERROR("...initializing powercubes not successful. error: %s", res.error_message.data.c_str());
      }
    }
    else
    {
      res.success.data = false;
      res.error_message.data = "powercubes already initialized";
      ROS_ERROR("...initializing powercubes not successful. error: %s",res.error_message.data.c_str());
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
      ROS_INFO("...stopping powercubes successful.");
    }
    else
    {
      res.success.data = false;
      res.error_message.data = pc_ctrl_->getErrorMessage();
      ROS_ERROR("...stopping powercubes not successful. error: %s", res.error_message.data.c_str());
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
    ROS_INFO("Recovering powercubes...");
    if (initialized_)
    {
      // stopping all arm movements
      if (pc_ctrl_->Recover())
      {
        res.success.data = true;
        ROS_INFO("...recovering powercubes successful.");
      }
      else
      {
        res.success.data = false;
        res.error_message.data = pc_ctrl_->getErrorMessage();
        ROS_ERROR("...recovering powercubes not successful. error: %s", res.error_message.data.c_str());
      }
    }
    else
    {
      res.success.data = false;
      res.error_message.data = "powercubes not initialized";
      ROS_ERROR("...recovering powercubes not successful. error: %s",res.error_message.data.c_str());
    }

    return true;
  }

  /*!
   * \brief Publishes the state of the powercube_chain as ros messages.
   *
   * Published to "/joint_states" as "sensor_msgs/JointState"
   * Published to "state" as "pr2_controllers_msgs/JointTrajectoryState"
   */
  void publishState()
  {
    if (initialized_)
    {
      ROS_INFO("publish state");

      pc_ctrl_->updateStates();

      sensor_msgs::JointState joint_state_msg;
      joint_state_msg.header.stamp = ros::Time::now();
      joint_state_msg.name = pc_params_->GetJointNames();
      joint_state_msg.position = pc_ctrl_->getPositions();
      joint_state_msg.velocity = pc_ctrl_->getVelocities();

      pr2_controllers_msgs::JointTrajectoryControllerState controller_state_msg;
      controller_state_msg.header.stamp = joint_state_msg.header.stamp;
      controller_state_msg.joint_names = pc_params_->GetJointNames();
      controller_state_msg.actual.positions = pc_ctrl_->getPositions();
      controller_state_msg.actual.velocities = pc_ctrl_->getVelocities();
      controller_state_msg.actual.accelerations = pc_ctrl_->getAccelerations();

      topicPub_JointState_.publish(joint_state_msg);
      topicPub_ControllerState_.publish(controller_state_msg);

      last_publish_time_ = ros::Time::now();
    }
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
  pc_node.getROSParameters();
  pc_node.getRobotDescriptionParameters();

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

  ros::Duration min_publish_duration;
  if (pc_node.n_.hasParam("min_publish_duration"))
  {
    double sec;
    pc_node.n_.getParam("min_publish_duration", sec);
    min_publish_duration.fromSec(sec);
  }
  else
  {
    min_publish_duration.fromSec(1 / frequency);
    ROS_WARN("Parameter min_publish_time not available, setting to default value: %f sec", min_publish_duration.toSec());
  }

  ros::Rate loop_rate(frequency); // Hz
  while (pc_node.n_.ok())
  {
    ROS_INFO("main");
    if ((ros::Time::now() - pc_node.last_publish_time_) >= min_publish_duration)
    {
      pc_node.publishState();
    }

    // sleep and waiting for messages, callbacks
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
