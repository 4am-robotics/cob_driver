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


#ifndef COB_BMS_DRIVER_NODE_H
#define COB_BMS_DRIVER_NODE_H

#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>

struct BmsParameter
{
    unsigned int offset;
    unsigned int length;

    bool is_signed;

    ros::Publisher publisher;

    diagnostic_msgs::KeyValue kv;

    virtual ~BmsParameter() {}

    virtual void update(const can::Frame &f) = 0;
    virtual void advertise(ros::NodeHandle &nh, const std::string &topic) = 0;

    typedef boost::shared_ptr<BmsParameter> Ptr;
};

class CobBmsDriverNode
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;

    typedef std::multimap<uint8_t, BmsParameter::Ptr> ConfigMap;

    //ROS parameters
    ConfigMap config_map_;  //holds all the information that is provided in the configuration file
    int poll_period_for_two_ids_in_ms_;
    std::string can_device_;
    int bms_id_to_poll_;
    ros::Timer updater_timer_;

    boost::mutex data_mutex_;

    //polling lists that contain CAN-ID(s) that are to be polled. Each CAN-ID corresponds to a group of BMS parameters
    std::vector<uint8_t> polling_list1_;
    std::vector<uint8_t> polling_list2_;
    std::vector<uint8_t>::iterator polling_list1_it_;
    std::vector<uint8_t>::iterator polling_list2_it_;

    //interface to send and recieve CAN frames
    can::ThreadedSocketCANInterface socketcan_interface_;

    //pointer to callback function to handle CAN frames from BMS
    can::CommInterface::FrameListenerConstSharedPtr frame_listener_;

    //diagnostics data received from BMS
    diagnostic_updater::DiagnosticStatusWrapper stat_;

    //function to get ROS parameters from parameter server
    bool getParams();

    //function to interpret the diagnostics XmlRpcValue and save data in config_map_
    bool loadConfigMap(XmlRpc::XmlRpcValue &diagnostics, std::vector<std::string> &topics);

    //helper function to evaluate poll period from given poll frequency
    void evaluatePollPeriodFrom(int poll_frequency);

    //function that goes through config_map_ and fills polling_list1_ and polling_list2_ with CAN-IDs.
    //If Topics are found on ROS Parameter Server, they are kept in a separate list (to be polled faster).
    //Otherwise, all CAN-ID are divided between both lists.
    void optimizePollingLists();

    //function that polls BMS (bms_id_to_poll_ is used here!).
    //Also, this function sleeps for time given by poll_period_for_two_ids_in_ms_ to ensure BMS is polled at the desired frequency
    void pollBmsForIds(const uint16_t first_id, const uint16_t second_id);

    //callback function to handle all types of frames received from BMS
    void handleFrames(const can::Frame &f);

    //updates the diagnostics data with the new data received from BMS
    void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    //calls update function of diagnostics_updater
    void diagnosticsTimerCallback(const ros::TimerEvent&);
public:

    //updater for diagnostics data
    diagnostic_updater::Updater updater_;

    CobBmsDriverNode();
    ~CobBmsDriverNode();

    //initlializes SocketCAN interface, saves data from ROS parameter server, loads polling lists and sets up diagnostic updater
    bool prepare();

    //cycles through polling lists and sends 2 ids at a time (one from each list) to the BMS
    void pollNextInLists();
};


#endif //COB_BMS_DRIVER_NODE_H
