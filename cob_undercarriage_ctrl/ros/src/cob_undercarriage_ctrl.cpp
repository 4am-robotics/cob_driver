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
 

//##################
//#### includes ####

// standard includes
#include <math.h>

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cob_msgs/EmergencyStopState.h>
#include <control_msgs/JointTrajectoryControllerState.h>

// external includes
#include <cob_undercarriage_ctrl/UndercarriageCtrlGeom.h>
#include <cob_utilities/IniFile.h>
//#include <cob_utilities/MathSup.h>

//####################
//#### node class ####
class NodeClass
{
  //
  public:
    // create a handle for this node, initialize node
    ros::NodeHandle n;

    // topics to publish
    ros::Publisher topic_pub_joint_state_cmd_;	// cmd issued for single joints of undercarriage
    ros::Publisher topic_pub_controller_joint_command_;
    ros::Publisher topic_pub_odometry_;			// calculated (measured) velocity, rotation and pose (odometry-based) for the robot
    tf::TransformBroadcaster tf_broadcast_odometry_;	// according transformation for the tf broadcaster

    // topics to subscribe, callback is called for new messages arriving
    ros::Subscriber topic_sub_CMD_pltf_twist_;	// issued command to be achieved by the platform
    ros::Subscriber topic_sub_EM_stop_state_;	// current emergency stop state (free, active, confirmed)
    ros::Subscriber topic_sub_drive_diagnostic_;// status of drive chain (initializing, error, normal)

    //subscribe to JointStates topic
    //ros::Subscriber topic_sub_joint_states_;
    ros::Subscriber topic_sub_joint_controller_states_;

    // diagnostic stuff
    diagnostic_updater::Updater updater_;

    // controller Timer
    ros::Timer timer_ctrl_step_;

    // member variables
    UndercarriageCtrlGeom * ucar_ctrl_;	// instantiate undercarriage controller
    std::string sIniDirectory;
    bool is_initialized_bool_;			// flag wether node is already up and running
    bool broadcast_tf_;			// flag wether to broadcast the tf from odom to base_link
    int drive_chain_diagnostic_;		// flag whether base drive chain is operating normal
    ros::Time last_time_;				// time Stamp for last odometry measurement
    ros::Time joint_state_odom_stamp_;	// time stamp of joint states used for current odometry calc
    double sample_time_, timeout_;
    double x_rob_m_, y_rob_m_, theta_rob_rad_; // accumulated motion of robot since startup
    int iwatchdog_;
    double 	vel_x_rob_last_, vel_y_rob_last_, vel_theta_rob_last_; //save velocities for better odom calculation
    double max_vel_trans_, max_vel_rot_;

    int m_iNumJoints;

    diagnostic_msgs::DiagnosticStatus diagnostic_status_lookup_; // used to access defines for warning levels

    // Constructor
    NodeClass()
    {
      // initialization of variables
      is_initialized_bool_ = false;
      broadcast_tf_ = true;
      iwatchdog_ = 0;
      last_time_ = ros::Time::now();
      sample_time_ = 0.020;
      x_rob_m_ = 0.0;
      y_rob_m_ = 0.0;
      theta_rob_rad_ = 0.0;
      vel_x_rob_last_ = 0.0;
      vel_y_rob_last_ = 0.0;
      vel_theta_rob_last_ = 0.0;
      // set status of drive chain to WARN by default
      drive_chain_diagnostic_ = diagnostic_status_lookup_.OK; //WARN; <- THATS FOR DEBUGGING ONLY!

      // Parameters are set within the launch file
      // read in timeout for watchdog stopping the controller.
      if (n.hasParam("timeout"))
      {
        n.getParam("timeout", timeout_);
        ROS_INFO("Timeout loaded from Parameter-Server is: %fs", timeout_);
      }
      else
      {
        ROS_WARN("No parameter timeout on Parameter-Server. Using default: 1.0s");
        timeout_ = 1.0;
      }
      if ( timeout_ < sample_time_ )
      {
        ROS_WARN("Specified timeout < sample_time. Setting timeout to sample_time = %fs", sample_time_);
        timeout_ = sample_time_;
      }

      // Read number of drives from iniFile and pass IniDirectory to CobPlatfCtrl.
      if (n.hasParam("IniDirectory"))
      {
        n.getParam("IniDirectory", sIniDirectory);
        ROS_INFO("IniDirectory loaded from Parameter-Server is: %s", sIniDirectory.c_str());
      }
      else
      {
        sIniDirectory = "Platform/IniFiles/";
        ROS_WARN("IniDirectory not found on Parameter-Server, using default value: %s", sIniDirectory.c_str());
      }

      if (n.hasParam("max_trans_velocity"))
      {
        n.getParam("max_trans_velocity", max_vel_trans_);
        ROS_INFO("Max translational velocity loaded from Parameter-Server is: %fs", max_vel_trans_);
      }
      else
      {
        ROS_WARN("No parameter max_trans_velocity on Parameter-Server. Using default: 1.1 m/s");
        max_vel_trans_ = 1.1;
      }
      if (n.hasParam("max_rot_velocity"))
      {
        n.getParam("max_rot_velocity", max_vel_rot_);
        ROS_INFO("Max rotational velocity loaded from Parameter-Server is: %fs", max_vel_rot_);
      }
      else
      {
        ROS_WARN("No parameter max_rot_velocity on Parameter-Server. Using default: 1.8 rad/s");
        max_vel_rot_ = 1.8;
      }
      if (n.hasParam("broadcast_tf"))
      {
        n.getParam("broadcast_tf", broadcast_tf_);
      }

      IniFile iniFile;
      iniFile.SetFileName(sIniDirectory + "Platform.ini", "PltfHardwareCoB3.h");
      iniFile.GetKeyInt("Config", "NumberOfMotors", &m_iNumJoints, true);

      ucar_ctrl_ = new UndercarriageCtrlGeom(sIniDirectory);


      // implementation of topics
      // published topics
      //topic_pub_joint_state_cmd_ = n.advertise<sensor_msgs::JointState>("joint_command", 1);
      topic_pub_controller_joint_command_ = n.advertise<control_msgs::JointTrajectoryControllerState> ("joint_command", 1);

      topic_pub_odometry_ = n.advertise<nav_msgs::Odometry>("odometry", 1);

      // subscribed topics
      topic_sub_CMD_pltf_twist_ = n.subscribe("command", 1, &NodeClass::topicCallbackTwistCmd, this);
      topic_sub_EM_stop_state_ = n.subscribe("/emergency_stop_state", 1, &NodeClass::topicCallbackEMStop, this);
      topic_sub_drive_diagnostic_ = n.subscribe("diagnostic", 1, &NodeClass::topicCallbackDiagnostic, this);



      //topic_sub_joint_states_ = n.subscribe("/joint_states", 1, &NodeClass::topicCallbackJointStates, this);
      topic_sub_joint_controller_states_ = n.subscribe("state", 1, &NodeClass::topicCallbackJointControllerStates, this);

      // diagnostics
      updater_.setHardwareID(ros::this_node::getName());
      updater_.add("initialization", this, &NodeClass::diag_init);

      //set up timer to cyclically call controller-step
      timer_ctrl_step_ = n.createTimer(ros::Duration(sample_time_), &NodeClass::timerCallbackCtrlStep, this);

    }

    // Destructor
    ~NodeClass()
    {
    }

    void diag_init(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
      if(is_initialized_bool_)
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "");
      else
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "");
      stat.add("Initialized", is_initialized_bool_);
    }

    // Listen for Pltf Cmds
    void topicCallbackTwistCmd(const geometry_msgs::Twist::ConstPtr& msg)
    {
      double vx_cmd_mms, vy_cmd_mms, w_cmd_rads;

      // check for NaN value in Twist message
      if(isnan(msg->linear.x) || isnan(msg->linear.y) || isnan(msg->angular.z)) {
        iwatchdog_ = 0;
        ROS_FATAL("Received NaN-value in Twist message. Stopping the robot.");
        // force platform velocity commands to zero;
        ucar_ctrl_->SetDesiredPltfVelocity(0.0, 0.0, 0.0, 0.0);
        ROS_DEBUG("Forced platform velocity commands to zero");
        return;
      }

      if( (fabs(msg->linear.x) > max_vel_trans_) || (fabs(msg->linear.y) > max_vel_trans_) || (fabs(msg->angular.z) > max_vel_rot_))
      {
        if(fabs(msg->linear.x) > max_vel_trans_)
        {
          ROS_DEBUG_STREAM("Recevied cmdVelX: " << msg->linear.x <<
              ", which is bigger than the maximal allowed translational velocity: " <<  max_vel_trans_ << " so stop the robot");
        }
        if(fabs(msg->linear.y) > max_vel_trans_)
        {
          ROS_DEBUG_STREAM("Recevied cmdVelY: " << msg->linear.x <<
              ", which is bigger than the maximal allowed translational velocity: " <<  max_vel_trans_ << " so stop the robot");
        }

        if(fabs(msg->angular.z) > max_vel_rot_)
        {
          ROS_DEBUG_STREAM("Recevied cmdVelTh: " << msg->angular.z <<
              ", which is bigger than the maximal allowed rotational velocity: " << max_vel_rot_ << " so stop the robot");
        }
        vx_cmd_mms = 0.0;
        vy_cmd_mms = 0.0;
        w_cmd_rads = 0.0;
      }
      else
      {
        // controller expects velocities in mm/s, ROS works with SI-Units -> convert
        // ToDo: rework Controller Class to work with SI-Units
        vx_cmd_mms = msg->linear.x*1000.0;
        vy_cmd_mms = msg->linear.y*1000.0;
        w_cmd_rads = msg->angular.z;
      }

      iwatchdog_ = 0;

      // only process if controller is already initialized
      if (is_initialized_bool_ && drive_chain_diagnostic_==diagnostic_status_lookup_.OK)
      {
        ROS_DEBUG("received new velocity command [cmdVelX=%3.5f,cmdVelY=%3.5f,cmdVelTh=%3.5f]",
            msg->linear.x, msg->linear.y, msg->angular.z);

        // Set desired value for Plattform Velocity to UndercarriageCtrl (setpoint setting)
        ucar_ctrl_->SetDesiredPltfVelocity(vx_cmd_mms, vy_cmd_mms, w_cmd_rads, 0.0);
        // ToDo: last value (0.0) is not used anymore --> remove from interface
      }
      else
      {
        // Set desired value for Plattform Velocity to zero (setpoint setting)
        ucar_ctrl_->SetDesiredPltfVelocity( 0.0, 0.0, 0.0, 0.0);
        // ToDo: last value (0.0) is not used anymore --> remove from interface
        ROS_DEBUG("Forced platform-velocity cmds to zero");
      }

    }

    // Listen for Emergency Stop
    void topicCallbackEMStop(const cob_msgs::EmergencyStopState::ConstPtr& msg)
    {
      int EM_state;
      EM_state = msg->emergency_state;

      if (EM_state == msg->EMFREE)
      {
        // Reset EM flag in Ctrlr
        if (is_initialized_bool_)
        {
          ucar_ctrl_->setEMStopActive(false);
          ROS_DEBUG("Undercarriage Controller EM-Stop released");
          // reset only done, when system initialized
          // -> allows to stop ctrlr during init, reset and shutdown
        }
      }
      else
      {
        ROS_DEBUG("Undercarriage Controller stopped due to EM-Stop");

        // Set desired value for Plattform Velocity to zero (setpoint setting)
        ucar_ctrl_->SetDesiredPltfVelocity( 0.0, 0.0, 0.0, 0.0);
        // ToDo: last value (0.0) is not used anymore --> remove from interface
        ROS_DEBUG("Forced platform-velocity cmds to zero");

        // Set EM flag and stop Ctrlr
        ucar_ctrl_->setEMStopActive(true);
      }
    }

    // Listens for status of underlying hardware (base drive chain)
    void topicCallbackDiagnostic(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg)
    {
      control_msgs::JointTrajectoryControllerState joint_state_cmd;

      // prepare joint_cmds for heartbeat (compose header)
      joint_state_cmd.header.stamp = ros::Time::now();
      //joint_state_cmd.header.frame_id = frame_id; //Where to get this id from?
      // ToDo: configure over Config-File (number of motors) and Msg
      // assign right size to JointState data containers
      //joint_state_cmd.set_name_size(m_iNumMotors);
      joint_state_cmd.desired.positions.resize(m_iNumJoints);
      joint_state_cmd.desired.velocities.resize(m_iNumJoints);
      //joint_state_cmd.desired.effort.resize(m_iNumJoints);
      joint_state_cmd.joint_names.push_back("fl_caster_r_wheel_joint");
      joint_state_cmd.joint_names.push_back("fl_caster_rotation_joint");
      joint_state_cmd.joint_names.push_back("bl_caster_r_wheel_joint");
      joint_state_cmd.joint_names.push_back("bl_caster_rotation_joint");
      joint_state_cmd.joint_names.push_back("br_caster_r_wheel_joint");
      joint_state_cmd.joint_names.push_back("br_caster_rotation_joint");
      joint_state_cmd.joint_names.push_back("fr_caster_r_wheel_joint");
      joint_state_cmd.joint_names.push_back("fr_caster_rotation_joint");
      joint_state_cmd.joint_names.resize(m_iNumJoints);

      // compose jointcmds
      for(int i=0; i<m_iNumJoints; i++)
      {
        joint_state_cmd.desired.positions[i] = 0.0;
        joint_state_cmd.desired.velocities[i] = 0.0;
        //joint_state_cmd.desired.effort[i] = 0.0;
      }

      // set status of underlying drive chain to member variable
      drive_chain_diagnostic_ = msg->level;

      // if controller is already started up ...
      if (is_initialized_bool_)
      {
        // ... but underlying drive chain is not yet operating normal
        if (drive_chain_diagnostic_ != diagnostic_status_lookup_.OK)
        {
          // halt controller
          ROS_DEBUG("drive chain not availlable: halt Controller");

          // Set EM flag to Ctrlr (resets internal states)
          ucar_ctrl_->setEMStopActive(true);

          // Set desired value for Plattform Velocity to zero (setpoint setting)
          ucar_ctrl_->SetDesiredPltfVelocity( 0.0, 0.0, 0.0, 0.0);
          // ToDo: last value (0.0) is not used anymore --> remove from interface
          ROS_DEBUG("Forced platform-velocity cmds to zero");

          // if is not Initializing
          if (drive_chain_diagnostic_ != diagnostic_status_lookup_.WARN)
          {
            // publish zero-vel. jointcmds to avoid Watchdogs stopping ctrlr
            // this is already done in CalcControlStep
          }
        }
      }
      // ... while controller is not initialized send heartbeats to keep motors alive
      else
      {
        // ... as soon as base drive chain is initialized
        if(drive_chain_diagnostic_ != diagnostic_status_lookup_.WARN)
        {
          // publish zero-vel. jointcmds to avoid Watchdogs stopping ctrlr
          topic_pub_controller_joint_command_.publish(joint_state_cmd);
        }
      }
    }

    void topicCallbackJointControllerStates(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg) {
      int num_joints;
      int iter_k, iter_j;
      std::vector<double> drive_joint_ang_rad, drive_joint_vel_rads, drive_joint_effort_NM;
      std::vector<double> steer_joint_ang_rad, steer_joint_vel_rads, steer_joint_effort_NM;

      joint_state_odom_stamp_ = msg->header.stamp;

      // copy configuration into vector classes
      num_joints = msg->joint_names.size();
      // drive joints
      drive_joint_ang_rad.assign(m_iNumJoints, 0.0);
      drive_joint_vel_rads.assign(m_iNumJoints, 0.0);
      drive_joint_effort_NM.assign(m_iNumJoints, 0.0);
      // steer joints
      steer_joint_ang_rad.assign(m_iNumJoints, 0.0);
      steer_joint_vel_rads.assign(m_iNumJoints, 0.0);
      steer_joint_effort_NM.assign(m_iNumJoints, 0.0);

      // init iterators
      iter_k = 0;
      iter_j = 0;

      for(int i = 0; i < num_joints; i++)
      {
        // associate inputs to according steer and drive joints
        // ToDo: specify this globally (Prms-File or config-File or via msg-def.)
        if(msg->joint_names[i] ==  "fl_caster_r_wheel_joint")
        {
          drive_joint_ang_rad[0] = msg->actual.positions[i];
          drive_joint_vel_rads[0] = msg->actual.velocities[i];
          //drive_joint_effort_NM[0] = msg->effort[i];
        }
        if(msg->joint_names[i] ==  "bl_caster_r_wheel_joint")
        {
          drive_joint_ang_rad[1] = msg->actual.positions[i];
          drive_joint_vel_rads[1] = msg->actual.velocities[i];
          //drive_joint_effort_NM[1] = msg->effort[i];
        }
        if(msg->joint_names[i] ==  "br_caster_r_wheel_joint")
        {
          drive_joint_ang_rad[2] = msg->actual.positions[i];
          drive_joint_vel_rads[2] = msg->actual.velocities[i];
          //drive_joint_effort_NM[2] = msg->effort[i];
        }
        if(msg->joint_names[i] ==  "fr_caster_r_wheel_joint")
        {
          drive_joint_ang_rad[3] = msg->actual.positions[i];
          drive_joint_vel_rads[3] = msg->actual.velocities[i];
          //drive_joint_effort_NM[3] = msg->effort[i];
        }
        if(msg->joint_names[i] ==  "fl_caster_rotation_joint")
        {
          steer_joint_ang_rad[0] = msg->actual.positions[i];
          steer_joint_vel_rads[0] = msg->actual.velocities[i];
          //steer_joint_effort_NM[0] = msg->effort[i];
        }
        if(msg->joint_names[i] ==  "bl_caster_rotation_joint")
        {
          steer_joint_ang_rad[1] = msg->actual.positions[i];
          steer_joint_vel_rads[1] = msg->actual.velocities[i];
          //steer_joint_effort_NM[1] = msg->effort[i];
        }
        if(msg->joint_names[i] ==  "br_caster_rotation_joint")
        {
          steer_joint_ang_rad[2] = msg->actual.positions[i];
          steer_joint_vel_rads[2] = msg->actual.velocities[i];
          //steer_joint_effort_NM[2] = msg->effort[i];
        }
        if(msg->joint_names[i] ==  "fr_caster_rotation_joint")
        {
          steer_joint_ang_rad[3] = msg->actual.positions[i];
          steer_joint_vel_rads[3] = msg->actual.velocities[i];
          //steer_joint_effort_NM[3] = msg->effort[i];
        }
      }

      // Set measured Wheel Velocities and Angles to Controler Class (implements inverse kinematic)
      ucar_ctrl_->SetActualWheelValues(drive_joint_vel_rads, steer_joint_vel_rads,
          drive_joint_ang_rad, steer_joint_ang_rad);


      // calculate odometry every time
      UpdateOdometry();

    }

    void timerCallbackCtrlStep(const ros::TimerEvent& e) {
      CalcCtrlStep();
    }

    // other function declarations
    // Initializes controller
    bool InitCtrl();
    // perform one control step, calculate inverse kinematics and publish updated joint cmd's (if no EMStop occurred)
    void CalcCtrlStep();
    // acquires the current undercarriage configuration from base_drive_chain
    // calculates odometry from current measurement values and publishes it via an odometry topic and the tf broadcaster
    void UpdateOdometry();
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
  // initialize ROS, spezify name of node
  ros::init(argc, argv, "undercarriage_ctrl");

  // construct nodeClass
  NodeClass nodeClass;

  // automatically do initializing of controller, because it's not directly depending any hardware components
  nodeClass.ucar_ctrl_->InitUndercarriageCtrl();
  nodeClass.is_initialized_bool_ = true;

  if( nodeClass.is_initialized_bool_ ) {
    nodeClass.last_time_ = ros::Time::now();
    ROS_INFO("Undercarriage control successfully initialized.");
  } else {
    ROS_FATAL("Undercarriage control initialization failed!");
    throw std::runtime_error("Undercarriage control initialization failed, check ini-Files!");
  }

  /*
     CALLBACKS being executed are:
     - actual motor values -> calculating direct kinematics and doing odometry (topicCallbackJointControllerStates)
     - timer callback -> calculate controller step at a rate of sample_time_ (timerCallbackCtrlStep)
     - other topic callbacks (diagnostics, command, em_stop_state)
     */
  ros::spin();

  return 0;
}

//##################################
//#### function implementations ####

// perform one control step, calculate inverse kinematics and publish updated joint cmd's (if no EMStop occurred)
void NodeClass::CalcCtrlStep()
{
  double vx_cmd_ms, vy_cmd_ms, w_cmd_rads, dummy;
  std::vector<double> drive_jointvel_cmds_rads, steer_jointvel_cmds_rads, steer_jointang_cmds_rad;
  control_msgs::JointTrajectoryControllerState joint_state_cmd;
  int j, k;
  iwatchdog_ += 1;

  // if controller is initialized and underlying hardware is operating normal
  if (is_initialized_bool_) //&& (drive_chain_diagnostic_ != diagnostic_status_lookup_.OK))
  {
    // as soon as (but only as soon as) platform drive chain is initialized start to send velocity commands
    // Note: topicCallbackDiagnostic checks whether drives are operating nominal.
    //       -> if warning or errors are issued target velocity is set to zero

    // perform one control step,
    // get the resulting cmd's for the wheel velocities and -angles from the controller class
    // and output the achievable pltf velocity-cmds (if velocity limits where exceeded)
    ucar_ctrl_->GetNewCtrlStateSteerDriveSetValues(drive_jointvel_cmds_rads,  steer_jointvel_cmds_rads, steer_jointang_cmds_rad, vx_cmd_ms, vy_cmd_ms, w_cmd_rads, dummy);
    // ToDo: adapt interface of controller class --> remove last values (not used anymore)

    // if drives not operating nominal -> force commands to zero
    if(drive_chain_diagnostic_ != diagnostic_status_lookup_.OK)
    {
      steer_jointang_cmds_rad.assign(m_iNumJoints, 0.0);
      steer_jointvel_cmds_rads.assign(m_iNumJoints, 0.0);
    }

    // convert variables to SI-Units
    vx_cmd_ms = vx_cmd_ms/1000.0;
    vy_cmd_ms = vy_cmd_ms/1000.0;

    // compose jointcmds
    // compose header
    joint_state_cmd.header.stamp = ros::Time::now();
    //joint_state_cmd.header.frame_id = frame_id; //Where to get this id from?
    // ToDo: configure over Config-File (number of motors) and Msg
    // assign right size to JointState data containers
    //joint_state_cmd.set_name_size(m_iNumMotors);
    joint_state_cmd.desired.positions.resize(m_iNumJoints);
    joint_state_cmd.desired.velocities.resize(m_iNumJoints);
    //joint_state_cmd.effort.resize(m_iNumJoints);
    joint_state_cmd.joint_names.push_back("fl_caster_r_wheel_joint");
    joint_state_cmd.joint_names.push_back("fl_caster_rotation_joint");
    joint_state_cmd.joint_names.push_back("bl_caster_r_wheel_joint");
    joint_state_cmd.joint_names.push_back("bl_caster_rotation_joint");
    joint_state_cmd.joint_names.push_back("br_caster_r_wheel_joint");
    joint_state_cmd.joint_names.push_back("br_caster_rotation_joint");
    joint_state_cmd.joint_names.push_back("fr_caster_r_wheel_joint");
    joint_state_cmd.joint_names.push_back("fr_caster_rotation_joint");
    joint_state_cmd.joint_names.resize(m_iNumJoints);

    // compose data body
    j = 0;
    k = 0;
    for(int i = 0; i<m_iNumJoints; i++)
    {
      if(iwatchdog_ < (int) std::floor(timeout_/sample_time_) )
      {
        // for steering motors
        if( i == 1 || i == 3 || i == 5 || i == 7) // ToDo: specify this via the Msg
        {
          joint_state_cmd.desired.positions[i] = steer_jointang_cmds_rad[j];
          joint_state_cmd.desired.velocities[i] = steer_jointvel_cmds_rads[j];
          //joint_state_cmd.effort[i] = 0.0;
          j = j + 1;
        }
        else
        {
          joint_state_cmd.desired.positions[i] = 0.0;
          joint_state_cmd.desired.velocities[i] = drive_jointvel_cmds_rads[k];
          //joint_state_cmd.effort[i] = 0.0;
          k = k + 1;
        }
      }
      else
      {
        joint_state_cmd.desired.positions[i] = 0.0;
        joint_state_cmd.desired.velocities[i] = 0.0;
        //joint_state_cmd.effort[i] = 0.0;
      }
    }

    // publish jointcmds
    topic_pub_controller_joint_command_.publish(joint_state_cmd);
  }

}

// calculates odometry from current measurement values
// and publishes it via an odometry topic and the tf broadcaster
void NodeClass::UpdateOdometry()
{
  double vel_x_rob_ms, vel_y_rob_ms, vel_rob_ms, rot_rob_rads, delta_x_rob_m, delta_y_rob_m, delta_theta_rob_rad;
  double dummy1, dummy2;
  double dt;
  ros::Time current_time;

  // if drive chain already initialized process joint data
  //if (drive_chain_diagnostic_ != diagnostic_status_lookup_.OK)
  if (is_initialized_bool_)
  {
    // Get resulting Pltf Velocities from Ctrl-Class (result of forward kinematics)
    // !Careful! Controller internally calculates with mm instead of m
    // ToDo: change internal calculation to SI-Units
    // ToDo: last values are not used anymore --> remove from interface
    ucar_ctrl_->GetActualPltfVelocity(delta_x_rob_m, delta_y_rob_m, delta_theta_rob_rad, dummy1,
        vel_x_rob_ms, vel_y_rob_ms, rot_rob_rads, dummy2);

    // convert variables to SI-Units
    vel_x_rob_ms = vel_x_rob_ms/1000.0;
    vel_y_rob_ms = vel_y_rob_ms/1000.0;
    delta_x_rob_m = delta_x_rob_m/1000.0;
    delta_y_rob_m = delta_y_rob_m/1000.0;

    ROS_DEBUG("Odmonetry delta is: x=%f, y=%f, th=%f", delta_x_rob_m, delta_y_rob_m, rot_rob_rads);
  }
  else
  {
    // otherwise set data (velocity and pose-delta) to zero
    vel_x_rob_ms = 0.0;
    vel_y_rob_ms = 0.0;
    delta_x_rob_m = 0.0;
    delta_y_rob_m = 0.0;
  }

  // calc odometry (from startup)
  // get time since last odometry-measurement
  current_time = ros::Time::now();
  dt = current_time.toSec() - last_time_.toSec();
  last_time_ = current_time;
  vel_rob_ms = sqrt(vel_x_rob_ms*vel_x_rob_ms + vel_y_rob_ms*vel_y_rob_ms);

  // calculation from ROS odom publisher tutorial http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Odom, using now midpoint integration
  x_rob_m_ = x_rob_m_ + ((vel_x_rob_ms+vel_x_rob_last_)/2.0 * cos(theta_rob_rad_) - (vel_y_rob_ms+vel_y_rob_last_)/2.0 * sin(theta_rob_rad_)) * dt;
  y_rob_m_ = y_rob_m_ + ((vel_x_rob_ms+vel_x_rob_last_)/2.0 * sin(theta_rob_rad_) + (vel_y_rob_ms+vel_y_rob_last_)/2.0 * cos(theta_rob_rad_)) * dt;
  theta_rob_rad_ = theta_rob_rad_ + rot_rob_rads * dt;
  //theta_rob_rad_ = theta_rob_rad_ + (rot_rob_rads+vel_theta_rob_last_)/2.0 * dt;

  vel_x_rob_last_ = vel_x_rob_ms;
  vel_y_rob_last_ = vel_y_rob_ms;
  vel_theta_rob_last_ = rot_rob_rads;


  // format data for compatibility with tf-package and standard odometry msg
  // generate quaternion for rotation
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_rob_rad_);

  if (broadcast_tf_ == true)
  {
    // compose and publish transform for tf package
    geometry_msgs::TransformStamped odom_tf;
    // compose header
    odom_tf.header.stamp = joint_state_odom_stamp_;
    odom_tf.header.frame_id = "odom_combined";
    odom_tf.child_frame_id = "base_footprint";
    // compose data container
    odom_tf.transform.translation.x = x_rob_m_;
    odom_tf.transform.translation.y = y_rob_m_;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = odom_quat;

    // publish the transform (for debugging, conflicts with robot-pose-ekf)
    tf_broadcast_odometry_.sendTransform(odom_tf);
  }

  // compose and publish odometry message as topic
  nav_msgs::Odometry odom_top;
  // compose header
  odom_top.header.stamp = joint_state_odom_stamp_;
  odom_top.header.frame_id = "odom_combined";
  odom_top.child_frame_id = "base_footprint";
  // compose pose of robot
  odom_top.pose.pose.position.x = x_rob_m_;
  odom_top.pose.pose.position.y = y_rob_m_;
  odom_top.pose.pose.position.z = 0.0;
  odom_top.pose.pose.orientation = odom_quat;
  for(int i = 0; i < 6; i++)
    odom_top.pose.covariance[i*6+i] = 0.1;

  // compose twist of robot
  odom_top.twist.twist.linear.x = vel_x_rob_ms;
  odom_top.twist.twist.linear.y = vel_y_rob_ms;
  odom_top.twist.twist.linear.z = 0.0;
  odom_top.twist.twist.angular.x = 0.0;
  odom_top.twist.twist.angular.y = 0.0;
  odom_top.twist.twist.angular.z = rot_rob_rads;
  for(int i = 0; i < 6; i++)
    odom_top.twist.covariance[6*i+i] = 0.1;

  // publish odometry msg
  topic_pub_odometry_.publish(odom_top);
}




