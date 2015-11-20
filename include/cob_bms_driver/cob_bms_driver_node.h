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
#include "std_msgs/String.h"	//check if needed
#include "std_msgs/Float64.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
 

class CobBmsDriverNode
{
	private:
	
		std::string can_device_;
		int bms_id_to_poll_;
		int poll_period_for_two_ids_in_ms_;

		std::vector<char> polling_list1_;
		std::vector<char> polling_list2_;
		std::vector<char>::iterator polling_list1_it_;
		std::vector<char>::iterator polling_list2_it_;
		
		can::ThreadedSocketCANInterface socketcan_interface_;
		can::CommInterface::FrameListener::Ptr frame_listener_;
		
		typedef std::map<char, std::vector<BmsParameter> > ConfigMap;	
		typedef std::vector<BmsParameter> BmsParameters;
		
		//config_map_ stores all the information that is provided in the config.yaml file
		ConfigMap config_map_;
		
		diagnostic_updater::DiagnosticStatusWrapper stat_;
		
		void getParams();
		void loadConfigMap(XmlRpc::XmlRpcValue, std::vector<std::string>);
		void createPublishersFor(std::vector<std::string>);
		void loadPollingLists();
		void evaluatePollPeriodFrom(int poll_frequency);
	
		//function that polls bms_id_to_poll_ for given parameters
		void pollBmsForIds(const char first_id, const char second_id);
		
		//handler for all frames
		void handleFrames(const can::Frame &f);

	public:
	
		ros::NodeHandle nh_;
		ros::NodeHandle nh_priv_;

		std::map<std::string, ros::Publisher> bms_diagnostics_publishers_;
		
		CobBmsDriverNode();
		~CobBmsDriverNode();
		bool prepare();
		void pollNextInLists();
		
		diagnostic_updater::Updater updater_;
		void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
		
	
	
};


#endif //COB_BMS_DRIVER_NODE_H
