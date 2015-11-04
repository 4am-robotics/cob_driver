#ifndef COB_BMS_DRIVER_NODE_H
#define COB_BMS_DRIVER_NODE_H

#include <sstream>
#include <vector>
#include <XmlRpcException.h>
#include <XmlRpcValue.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "socketcan_interface/socketcan.h"
#include "socketcan_interface/threading.h"
#include "socketcan_interface/string.h"
#include "cob_bms_driver/cob_bms_driver.h"
#include "cob_bms_driver/bms_variable.h"

class CobBmsDriverNode
{
	private:
	
		std::vector<int> param_list1_;
		std::vector<int> param_list2_;
		std::vector<int>::iterator param_list1_it_;
		std::vector<int>::iterator param_list2_it_;
		BmsDriver bms_driver_;
		std::vector<std::map<int, BmsVariable> > config_map_vec_;
		std::vector<std::string> topics_;
		
		void loadConfigMap(XmlRpc::XmlRpcValue);
		void loadTopics(std::vector<std::string>);
		void loadParameters();
		void loadParameterLists();

	public:
	
		ros::NodeHandle nh_;
		
		CobBmsDriverNode();
		~CobBmsDriverNode();
		bool prepare();
		void pollNextInParamLists();
};


#endif //COB_BMS_DRIVER_NODE_H
