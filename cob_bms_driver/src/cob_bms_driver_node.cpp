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


#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Bool.h>
#include <XmlRpcException.h>

#include <stdint.h>
#include <endian.h>
#include <boost/format.hpp>

#include <cob_bms_driver/cob_bms_driver_node.h>

using boost::make_shared;

//template to be used to read CAN frames received from the BMS
template<int N> void big_endian_to_host(const void* in, void* out);
template<> void big_endian_to_host<1>(const void* in, void* out){ *(uint8_t*)out = *(uint8_t*)in;}
template<> void big_endian_to_host<2>(const void* in, void* out){ *(uint16_t*)out = be16toh(*(uint16_t*)in);}
template<> void big_endian_to_host<4>(const void* in, void* out){ *(uint32_t*)out = be32toh(*(uint32_t*)in);}
template<> void big_endian_to_host<8>(const void* in, void* out){ *(uint64_t*)out = be64toh(*(uint64_t*)in);}

template<typename T> T read_value(const can::Frame &f, uint8_t offset){
    T res;
    big_endian_to_host<sizeof(T)>(&f.data[offset], &res);
    return res;
}
template<typename T> bool readTypedValue(const can::Frame &f, const BmsParameter &param, T &data){
    switch (param.length)
    {
    case 1:
        data = param.is_signed ? read_value<int8_t> (f, param.offset) : read_value<uint8_t> (f, param.offset);
        break;

    case 2:
        data = param.is_signed ? read_value<int16_t> (f, param.offset) : read_value<uint16_t> (f, param.offset);
        break;

    case 4:
        data = param.is_signed ? read_value<int32_t> (f, param.offset) : read_value<uint32_t> (f, param.offset);
        break;

    default:
            ROS_WARN_STREAM("Unknown length of BmsParameter: " << param.kv.key << ". Cannot read data!");
            return false;
    }
    return true;
}

template<typename T> struct TypedBmsParameter : BmsParameter {
    T msg_;

    void publish(){
        //if the BmsParameter is a topic, publish data to the topic
        if (static_cast<void*>(publisher))
        {
            publisher.publish(msg_);
        }
    }
    virtual void advertise(ros::NodeHandle &nh, const std::string &topic){
        publisher = nh.advertise<T> (topic, 1, true);
    }
};

struct FloatBmsParameter : TypedBmsParameter<std_msgs::Float64> {
    double factor;
    FloatBmsParameter(double factor) : factor(factor) {}
    void update(const can::Frame &f){
        readTypedValue(f, *this, msg_.data);
        msg_.data *= factor;

        //save data for diagnostics updater (and round to two digits for readability)
        std::stringstream sstream;
        sstream << std::setprecision(2) << msg_.data;
        kv.value = sstream.str();
        publish();
    }
};

struct IntBmsParameter : TypedBmsParameter<std_msgs::Int64> {
    IntBmsParameter() {}
    void update(const can::Frame &f){
        readTypedValue(f, *this, msg_.data);

        kv.value = (boost::format("%lld") % msg_.data).str();
        publish();
    }
};

struct UIntBmsParameter : TypedBmsParameter<std_msgs::UInt64> {
    UIntBmsParameter() {}
    void update(const can::Frame &f){
        readTypedValue(f, *this, msg_.data);

        kv.value = (boost::format("%llu") % msg_.data).str();
        publish();
    }
};

struct BooleanBmsParameter : TypedBmsParameter<std_msgs::Bool> {
    int bit_mask;
    BooleanBmsParameter(int bit_mask) : bit_mask(bit_mask) { is_signed = true; }
    void update(const can::Frame &f){
        int value;
        readTypedValue(f, *this, value);
        msg_.data = (value & bit_mask) == bit_mask;

        kv.value = msg_.data ? "True" : "False";
        publish();
    }
};

CobBmsDriverNode::CobBmsDriverNode()
: nh_priv_("~")
{}

CobBmsDriverNode::~CobBmsDriverNode()
{
    socketcan_interface_.shutdown();
}

//initlializes SocketCAN interface, saves data from ROS parameter server, loads polling lists and sets up diagnostic updater
bool CobBmsDriverNode::prepare()
{
    //reads parameters from ROS parameter server and saves them in respective member variable: config_map_, poll_period_for_two_ids_in_ms_, can_device_, bms_id_to_poll_
    if (!getParams())
    {
        ROS_ERROR("Could not prepare driver for start up");
        return false;
    }

    optimizePollingLists();

    //initalize polling lists iterators
    polling_list1_it_ = polling_list1_.begin();
    polling_list2_it_ = polling_list2_.begin();

    updater_.setHardwareID("bms");
    updater_.add("cob_bms_dagnostics_updater", this, &CobBmsDriverNode::produceDiagnostics);

    updater_timer_ = nh_.createTimer(ros::Duration(updater_.getPeriod()), &CobBmsDriverNode::diagnosticsTimerCallback, this);

    //initialize the socketcan interface
    if(!socketcan_interface_.init(can_device_, false)) {
        ROS_ERROR("cob_bms_driver initialization failed");
        return false;
    }

    //create listener for CAN frames
    frame_listener_  = socketcan_interface_.createMsgListener(can::CommInterface::FrameDelegate(this, &CobBmsDriverNode::handleFrames));

    return true;
}

//function to get ROS parameters from parameter server
bool CobBmsDriverNode::getParams()
{
    //local declarations
    XmlRpc::XmlRpcValue diagnostics;
    std::vector <std::string> topics;
    int poll_frequency_hz;

    if (!nh_priv_.getParam("topics", topics))
    {
        ROS_INFO_STREAM("Did not find \"topics\" on parameter server");
    }
    if (topics.empty())
    {
        ROS_INFO("Topic list is empty. No publisher created");
    }

    if (!nh_priv_.getParam("diagnostics", diagnostics))
    {
        ROS_ERROR_STREAM("Did not find \"diagnostics\" on parameter server");
        return false;
    }
    try {
        if(!loadConfigMap(diagnostics, topics)) return false;
    }
    catch(XmlRpc::XmlRpcException &e){
        ROS_ERROR_STREAM("Could not parse 'diagnostics': "<< e.getMessage());
        return false;
    }

    if (!topics.empty())
    {
        for(std::vector<std::string>::iterator it = topics.begin(); it != topics.end(); ++it){
            ROS_ERROR_STREAM("Could not find entry for topic '" << *it << "'.");
        }
        return false;
    }

    if (!nh_priv_.getParam("can_device", can_device_))
    {
        ROS_INFO_STREAM("Did not find \"can_device\" on parameter server. Using default value: can0");
        can_device_ = "can0";
    }

    if (!nh_priv_.getParam("bms_id_to_poll", bms_id_to_poll_))
    {
        ROS_INFO_STREAM("Did not find \"bms_id_to_poll\" on parameter server. Using default value: 0x200");
        bms_id_to_poll_ = 0x200;
    }

    if (!nh_priv_.getParam("poll_frequency_hz", poll_frequency_hz))
    {
        ROS_INFO_STREAM("Did not find \"poll_frequency\" on parameter server. Using default value: 20 Hz");
        poll_frequency_hz = 20;
    }
    evaluatePollPeriodFrom(poll_frequency_hz);
    return true;
}

//function to interpret the diagnostics XmlRpcValue and save data in config_map_
bool CobBmsDriverNode::loadConfigMap(XmlRpc::XmlRpcValue &diagnostics, std::vector<std::string> &topics)
{
    //for each id in list of ids
    for (size_t i = 0; i < diagnostics.size(); ++i)
    {
        uint8_t id;
        XmlRpc::XmlRpcValue config = diagnostics[i];

        if(!config.hasMember("id")) {
            ROS_ERROR_STREAM("diagnostics[" << i << "]: id is missing.");
            return false;
        }
        if(!config.hasMember("fields")) {
            ROS_ERROR_STREAM("diagnostics[" << i << "]: fields is missing.");
            return false;
        }
        id = static_cast<uint8_t>(static_cast<int>(config["id"]));

        XmlRpc::XmlRpcValue fields = config["fields"];
        bool publishes = false;

        for(int32_t j=0; j<fields.size(); ++j)
        {
            XmlRpc::XmlRpcValue field = fields[j];
            if(!field.hasMember("name")){
                ROS_ERROR_STREAM("diagnostics[" << i << "]: fields[" << j << "]: name is missing.");
                return false;
            }
            std::string name = static_cast<std::string>(field["name"]);

            if(!field.hasMember("len")){
                ROS_ERROR_STREAM("diagnostics[" << i << "]: fields[" << j << "]: len is missing.");
                return false;
            }
            int len = static_cast<int>(field["len"]);

            BmsParameter::Ptr entry;
            if(field.hasMember("bit_mask")){
                int bit_mask = static_cast<int>(field["bit_mask"]);
                if(bit_mask & ~((1<<(len*8))-1)){
                    ROS_ERROR_STREAM("diagnostics[" << i << "]: fields[" << j << "]: bit_mask does fit not into type of length " << len);
                    return false;
                }
                entry = make_shared<BooleanBmsParameter>(bit_mask);
                entry->kv.key = name;
            }else{
                if(!field.hasMember("is_signed")){
                    ROS_ERROR_STREAM("diagnostics[" << i << "]: fields[" << j << "]: is_signed is missing.");
                    return false;
                }
                if(field.hasMember("factor")){
                    double factor = 1.0;
                    if(field.hasMember("factor")){
                        factor = static_cast<double>(field["factor"]);
                    }

                    entry = make_shared<FloatBmsParameter>(factor);

                    entry->is_signed = static_cast<bool>(field["is_signed"]);

                    if(field.hasMember("unit")){
                        entry->kv.key = name + "[" + static_cast<std::string>(field["unit"]) + "]";
                    }else{
                        entry->kv.key = name;
                    }
                }else{
                    if(static_cast<bool>(field["is_signed"])){
                        entry = make_shared<IntBmsParameter>();
                    }else{
                        entry = make_shared<UIntBmsParameter>();
                    }
                    entry->is_signed = static_cast<bool>(field["is_signed"]);
                    entry->kv.key = name;
                }
            }

            if(!field.hasMember("offset")){
                ROS_ERROR_STREAM("diagnostics[" << i << "]: fields[" << j << "]: offset is missing.");
                return false;
            }
            entry->offset = static_cast<int>(field["offset"]);

            entry->length = len;

            std::vector<std::string>::iterator topic_it = find(topics.begin(), topics.end(), name);
            if(topic_it != topics.end()){
                entry->advertise(nh_priv_, name);
                topics.erase(topic_it);
                publishes = true;
                ROS_INFO_STREAM("Created publisher for: " << name);
            }

            config_map_.insert(std::make_pair(id, entry));

        }
        if(publishes) polling_list1_.push_back(id);
        else polling_list2_.push_back(id);

        ROS_INFO_STREAM("Got "<< fields.size() << " BmsParameter(s) with CAN-ID: 0x" << std::hex << (unsigned int) id << std::dec);
    }
    return true;
}

//helper function to evaluate poll period from given poll frequency
void CobBmsDriverNode::evaluatePollPeriodFrom(int poll_frequency_hz)
{
    //check the validity of given poll_frequency_hz
    if ((poll_frequency_hz < 0) && (poll_frequency_hz > 20))
    {
        ROS_WARN_STREAM("Invalid ROS parameter value: poll_frequency_hz = "<< poll_frequency_hz << ". Setting poll_frequency_hz to 20 Hz");
        poll_frequency_hz = 20;
    }
    //now evaluate and save poll period
    poll_period_for_two_ids_in_ms_ = (1/double(poll_frequency_hz))*1000;
    ROS_INFO_STREAM("Evaluated polling period: "<< poll_period_for_two_ids_in_ms_ << " ms");
}

//function that goes through config_map_ and fills polling_list1_ and polling_list2_. If topics are found on ROS Parameter Server, they are kept in list1 otherwise, all parameter id are divided between both lists.
void CobBmsDriverNode::optimizePollingLists()
{
    if(polling_list1_.size() == 0){ // no topics, so just distribute topics
        while(polling_list1_.size() < polling_list2_.size()){
            polling_list1_.push_back(polling_list2_.back());
            polling_list2_.pop_back();
        }
    }else{
        while(polling_list1_.size() > polling_list2_.size()){
            polling_list2_.push_back(polling_list1_.back());
            polling_list1_.pop_back();
        }
    }
    ROS_INFO_STREAM("Loaded \'"<< polling_list1_.size() << "\' CAN-ID(s) in polling_list1_ and \'"<< polling_list2_.size() <<"\' CAN-ID(s) in polling_list2_");
}

//function that polls BMS for given ids
void CobBmsDriverNode::pollBmsForIds(const uint16_t first_id, const uint16_t second_id)
{
    can::Frame f(can::Header(bms_id_to_poll_,false,false,false),4);
    f.data[0] = first_id >> 8;  //high_byte
    f.data[1] = first_id & 0xff;    //low_byte
    f.data[2] = second_id >> 8;
    f.data[3] = second_id & 0xff;

    socketcan_interface_.send(f);

    boost::this_thread::sleep_for(boost::chrono::milliseconds(poll_period_for_two_ids_in_ms_));
}

//cycles through polling lists and sends 2 ids at a time (one from each list) to the BMS
void CobBmsDriverNode::pollNextInLists()
{
    //restart if reached the end of polling lists
    if (polling_list1_it_ == polling_list1_.end()) polling_list1_it_ = polling_list1_.begin();
    if (polling_list2_it_ == polling_list2_.end()) polling_list2_it_ = polling_list2_.begin();
    //clear stat_, so that it can be refilled with new data
    stat_.clear();

    uint16_t first_id = (polling_list1_it_ == polling_list1_.end()) ? 0 : (*polling_list1_it_ | 0x0100);
    uint16_t second_id = (polling_list2_it_ == polling_list2_.end()) ? 0 : (*polling_list2_it_ | 0x0100);

    ROS_DEBUG_STREAM("polling BMS for CAN-IDs: 0x" << std::hex << (int)first_id << " and 0x" << (int) second_id << std::dec);

    pollBmsForIds(first_id,second_id);

    //increment iterators for next poll
    if (!polling_list1_.empty()) ++polling_list1_it_;
    if (!polling_list2_.empty()) ++polling_list2_it_;
}

//callback function to handle all types of frames received from BMS
void CobBmsDriverNode::handleFrames(const can::Frame &f)
{
    boost::mutex::scoped_lock lock(data_mutex_);

    //id to find in config map
    std::pair<ConfigMap::iterator, ConfigMap::iterator>  range = config_map_.equal_range(f.id);

    for (; range.first != range.second; ++range.first)
    {
        range.first->second->update(f);
    }
}

//updates the diagnostics data with the new data received from BMS
void CobBmsDriverNode::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    boost::mutex::scoped_lock lock(data_mutex_);

    can::State state = socketcan_interface_.getState();
    stat.add("error_code", state.error_code);
    stat.add("can_error_code", state.internal_error);
    switch (state.driver_state)
    {
    case can::State::closed:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Driver State: Closed");
        break;

    case can::State::open:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Driver State: Opened");
        break;

    case can::State::ready:
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Driver State: Ready");
        break;

    default:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Driver State: Unknown");
        break;
    }

    for (ConfigMap::iterator cm_it = config_map_.begin(); cm_it != config_map_.end(); ++cm_it)
    {
                stat.values.push_back(cm_it->second->kv);
    }
}

void CobBmsDriverNode::diagnosticsTimerCallback(const ros::TimerEvent& event)
{
    //update diagnostics
    updater_.update();
    if(!socketcan_interface_.getState().isReady()){
        ROS_ERROR("Restarting BMS socketcan");
        socketcan_interface_.shutdown();
        socketcan_interface_.recover();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bms_driver_node");

    CobBmsDriverNode cob_bms_driver_node;

    if (!cob_bms_driver_node.prepare()) return 1;

    ROS_INFO("Started polling BMS...");
    while (ros::ok())
    {
        cob_bms_driver_node.pollNextInLists();
        ros::spinOnce();
    }
    return 0;
}
