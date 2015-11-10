#ifndef COB_BMS_DRIVER_NODE_H
#define COB_BMS_DRIVER_NODE_H

#include <sstream>
#include <vector>

#include <stdint.h>
#include <endian.h>
#include "socketcan_interface/socketcan.h"
#include "socketcan_interface/threading.h"
#include "socketcan_interface/string.h"

//#include "cob_bms_driver/cob_bms_driver.h"
#include "cob_bms_driver/bms_parameter.h"

#include <XmlRpcException.h>
#include <XmlRpcValue.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

class CobBmsDriverNode
{
	private:
	
		//TODO
		unsigned int bms_id_;

		std::vector<char> param_list1_;
		std::vector<char> param_list2_;
		std::vector<char>::iterator param_list1_it_;
		std::vector<char>::iterator param_list2_it_;
		
		can::ThreadedSocketCANInterface socketcan_interface_;
		can::CommInterface::FrameListener::Ptr frame_listener_;
		
		typedef std::map<char, std::vector<BmsParameter> > ConfigMap;
		typedef std::vector<BmsParameter> BmsParameters;
		
		ConfigMap config_map_;
		std::vector<std::string> topics_;
		
		void loadConfigMap(XmlRpc::XmlRpcValue, bool);
		void loadTopics(std::vector<std::string>);
		void loadParameters();
	
		//function that polls all batteries (i.e. at CAN ID: 0x200) for two parameters at a time
		bool pollBmsforParameters(const char first_parameter_id, const char second_parameter_id /*, void (*callback)(std::string&)*/);
		
		//handler for all frames
		void handleFrames(const can::Frame &f);

	public:
	
		ros::NodeHandle nh_;
		
		CobBmsDriverNode();
		~CobBmsDriverNode();
		bool prepare();
		void pollNextInParamLists();
	
	
};


#endif //COB_BMS_DRIVER_NODE_H
