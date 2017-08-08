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
//--

// ROS includes
#include <ros/ros.h>
#include <cob_relayboard/SerRelayBoard.h>

// ROS message includes
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <cob_msgs/EmergencyStopState.h>

// ROS service includes
//--

// external includes
//--

//####################
//#### node class ####
class NodeClass
{
  //
public:
  // create a handle for this node, initialize node
  ros::NodeHandle n;
  ros::NodeHandle n_priv;

  // topics to publish
  ros::Publisher topicPub_isEmergencyStop;
  ros::Publisher topicPub_Voltage;
  // topics to subscribe, callback is called for new messages arriving
  // --

  // Constructor
  NodeClass()
  {
    n = ros::NodeHandle();
    n_priv = ros::NodeHandle("~");

    topicPub_isEmergencyStop = n.advertise<cob_msgs::EmergencyStopState>("emergency_stop_state", 1);
    topicPub_Voltage = n.advertise<std_msgs::Float64>("voltage", 1);

    // Make sure member variables have a defined state at the beginning
    EM_stop_status_ = ST_EM_FREE;
    relayboard_available = false;
    relayboard_online = false;
    relayboard_timeout_ = 2.0;
    protocol_version_ = 1;
    duration_for_EM_free_ = ros::Duration(1);
  }

  // Destructor
  ~NodeClass()
  {
    delete m_SerRelayBoard;
  }

  void sendEmergencyStopStates();
  void sendBatteryVoltage();
  int init();

private:
  std::string sComPort;
  SerRelayBoard * m_SerRelayBoard;

  int EM_stop_status_;
  ros::Duration duration_for_EM_free_;
  ros::Time time_of_EM_confirmed_;
  double relayboard_timeout_;
  int protocol_version_;

  ros::Time time_last_message_received_;
  bool relayboard_online; //the relayboard is sending messages at regular time
  bool relayboard_available; //the relayboard has sent at least one message -> publish topic

  // possible states of emergency stop
  enum
    {
      ST_EM_FREE = 0,
      ST_EM_ACTIVE = 1,
      ST_EM_CONFIRMED = 2
    };

  int requestBoardStatus();
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
  // initialize ROS, spezify name of node
  ros::init(argc, argv, "cob_relayboard_node");

  NodeClass node;
  if(node.init() != 0) return 1;

  ros::Rate r(20); //Cycle-Rate: Frequency of publishing EMStopStates
  while(node.n.ok())
    {
      node.sendEmergencyStopStates();

      ros::spinOnce();
      r.sleep();
    }

  return 0;
}

//##################################
//#### function implementations ####

int NodeClass::init()
{
  if (n_priv.hasParam("ComPort"))
    {
      n_priv.getParam("ComPort", sComPort);
      ROS_INFO("Loaded ComPort parameter from parameter server: %s",sComPort.c_str());
    }
  else
    {
      sComPort ="/dev/ttyUSB0";
      ROS_WARN("ComPort Parameter not available, using default Port: %s",sComPort.c_str());
    }

  n_priv.param("relayboard_timeout", relayboard_timeout_, 2.0);
  n_priv.param("protocol_version", protocol_version_, 1);

  m_SerRelayBoard = new SerRelayBoard(sComPort, protocol_version_);
  ROS_INFO("Opened Relayboard at ComPort = %s", sComPort.c_str());

  m_SerRelayBoard->init();

  // Init member variable for EM State
  EM_stop_status_ = ST_EM_ACTIVE;
  duration_for_EM_free_ = ros::Duration(1);

  return 0;
}

int NodeClass::requestBoardStatus() {
  int ret;

  // Request Status of RelayBoard
  ret = m_SerRelayBoard->sendRequest();
  if(ret != SerRelayBoard::NO_ERROR) {
    ROS_ERROR("Error in sending message to Relayboard over SerialIO, lost bytes during writing");
  }

  ret = m_SerRelayBoard->evalRxBuffer();
  if(ret==SerRelayBoard::NOT_INITIALIZED) {
    ROS_ERROR("Failed to read relayboard data over Serial, the device is not initialized");
    relayboard_online = false;
  } else if(ret==SerRelayBoard::NO_MESSAGES) {
    ROS_ERROR("For a long time, no messages from RelayBoard have been received, check com port!");
    if(time_last_message_received_.toSec() - ros::Time::now().toSec() > relayboard_timeout_) {relayboard_online = false;}
  } else if(ret==SerRelayBoard::TOO_LESS_BYTES_IN_QUEUE) {
    //ROS_ERROR("Relayboard: Too less bytes in queue");
  } else if(ret==SerRelayBoard::CHECKSUM_ERROR) {
    ROS_ERROR("A checksum error occurred while reading from relayboard data");
  } else if(ret==SerRelayBoard::NO_ERROR) {
    relayboard_online = true;
    relayboard_available = true;
    time_last_message_received_ = ros::Time::now();
  }

  return 0;
}

void NodeClass::sendBatteryVoltage()
{
  std_msgs::Float64 voltage;
  voltage.data = m_SerRelayBoard->getBatteryVoltage()/1000.0; //normalize from mV to V
  topicPub_Voltage.publish(voltage);
}

void NodeClass::sendEmergencyStopStates()
{
  requestBoardStatus();

  if(!relayboard_available) return;

  sendBatteryVoltage();


  bool EM_signal;
  ros::Duration duration_since_EM_confirmed;
  cob_msgs::EmergencyStopState EM_msg;

  // assign input (laser, button) specific EM state TODO: Laser and Scanner stop can't be read independently (e.g. if button is stop --> no informtion about scanner, if scanner ist stop --> no informtion about button stop)
  EM_msg.emergency_button_stop = m_SerRelayBoard->isEMStop();
  EM_msg.scanner_stop = m_SerRelayBoard->isScannerStop();

  // determine current EMStopState
  EM_signal = (EM_msg.emergency_button_stop || EM_msg.scanner_stop);

  switch (EM_stop_status_)
    {
    case ST_EM_FREE:
      {
      if (EM_signal == true)
        {
          ROS_INFO("Emergency stop was issued");
          EM_stop_status_ = EM_msg.EMSTOP;
        }
      break;
      }
    case ST_EM_ACTIVE:
      {
      if (EM_signal == false)
        {
          ROS_INFO("Emergency stop was confirmed");
          EM_stop_status_ = EM_msg.EMCONFIRMED;
          time_of_EM_confirmed_ = ros::Time::now();
        }
      break;
      }
    case ST_EM_CONFIRMED:
      {
      if (EM_signal == true)
        {
          ROS_INFO("Emergency stop was issued");
          EM_stop_status_ = EM_msg.EMSTOP;
        }
      else
        {
          duration_since_EM_confirmed = ros::Time::now() - time_of_EM_confirmed_;
          if( duration_since_EM_confirmed.toSec() > duration_for_EM_free_.toSec() )
            {
            ROS_INFO("Emergency stop released");
            EM_stop_status_ = EM_msg.EMFREE;
            }
        }
      break;
      }
    };


  EM_msg.emergency_state = EM_stop_status_;

  //publish EM-Stop-Active-messages, when connection to relayboard got cut
  if(relayboard_online == false) {
    EM_msg.emergency_state = EM_msg.EMSTOP;
  }
  topicPub_isEmergencyStop.publish(EM_msg);
}
