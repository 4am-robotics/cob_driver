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
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
 

class CobBmsDriverNode
{
	private:
	
		//TODO
		std::string can_device_;
		int can_id_to_poll_;
		int poll_period_for_two_parameters_in_ms_;
		

		std::vector<char> param_list1_;
		std::vector<char> param_list2_;
		std::vector<char>::iterator param_list1_it_;
		std::vector<char>::iterator param_list2_it_;
		
		can::ThreadedSocketCANInterface socketcan_interface_;
		can::CommInterface::FrameListener::Ptr frame_listener_;
		
		typedef std::map<char, std::vector<BmsParameter> > ConfigMap;	
		typedef std::vector<BmsParameter> BmsParameters;
		
		//config_map_ stores all the information that is provided in the config.yaml file
		ConfigMap config_map_;
		std::vector<std::string> topics_; 	//TODO: check if this is actually needed
		
		diagnostic_updater::DiagnosticStatusWrapper stat_;
		
		void getRosParameters();
		void loadConfigMap(XmlRpc::XmlRpcValue, std::vector<std::string>);
		void setTopics(std::vector<std::string>);
		void loadParameterLists();
	
		//function that polls all batteries (i.e. at CAN ID: 0x200) for two parameters at a time
		bool pollBmsforParameters(const char first_parameter_id, const char second_parameter_id /*, void (*callback)(std::string&)*/);
		
		//handler for all frames
		void handleFrames(const can::Frame &f);
		
		//helper functions
		bool isAlsoTopic(std::string paramater_name);

	public:
	
		ros::NodeHandle nh_;
		ros::NodeHandle nh_priv_;
		
		CobBmsDriverNode();
		~CobBmsDriverNode();
		bool prepare();
		void pollNextInParamLists();
		
		diagnostic_updater::Updater updater_;
		void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
		
	
	
};


#endif //COB_BMS_DRIVER_NODE_H
