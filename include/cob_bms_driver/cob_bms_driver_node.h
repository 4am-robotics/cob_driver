#ifndef COB_BMS_DRIVER_NODE_H
#define COB_BMS_DRIVER_NODE_H

#include <sstream>
#include <vector>
#include <stdint.h>
#include <endian.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <XmlRpcException.h>
#include <XmlRpcValue.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include "socketcan_interface/socketcan.h"
#include "socketcan_interface/threading.h"
#include "socketcan_interface/string.h"

#include "cob_bms_driver/bms_parameter.h"

class CobBmsDriverNode
{
	private:
		//config_map_ stores all the information that is provided in the config.yaml file
		typedef std::map<char, std::vector<BmsParameter> > ConfigMap;	
		ConfigMap config_map_;

		//lists that contain diagnostics ids that are to be polled, each field/element in a diagnostic id is a BmsParameter
		std::vector<char> polling_list1_;
		std::vector<char> polling_list2_;
		std::vector<char>::iterator polling_list1_it_;
		std::vector<char>::iterator polling_list2_it_;

		std::string can_device_;
		int bms_id_to_poll_;
		int poll_period_for_two_ids_in_ms_;

		diagnostic_updater::DiagnosticStatusWrapper stat_;

		can::ThreadedSocketCANInterface socketcan_interface_;
		can::CommInterface::FrameListener::Ptr frame_listener_;
		
		//function to get parameters from parameter server
		void getParams();

		//function to interpret the diagnostics XmlRpcValue and save data in config_map_
		void loadConfigMap(XmlRpc::XmlRpcValue diagnostics, std::vector<std::string> topics);

		//function to create a publisher for each Topic that is listed in the configuration file
		void createPublishersFor(std::vector<std::string> topics);

		//function to fill polling_list1_ and polling_list2_. Rule for filling the lists is: a diagnostic id that contains a Topic, should be kept in a smaller list
		void loadPollingLists();

		//helper function to evaluate poll period from given poll frequency
		void evaluatePollPeriodFrom(int poll_frequency);
	
		//function to polls bms_id_to_poll_ for given diagnostic ids
		void pollBmsForIds(const char first_id, const char second_id);
		
		//callback function to handle all types of frames received from BMS
		void handleFrames(const can::Frame &f);

	public:
	
		ros::NodeHandle nh_;
		ros::NodeHandle nh_priv_;
		diagnostic_updater::Updater updater_;
		std::map<std::string, ros::Publisher> bms_diagnostics_publishers_;
		
		CobBmsDriverNode();
		~CobBmsDriverNode();

		bool prepare();
		void pollNextInLists();
		
		void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);	
};


#endif //COB_BMS_DRIVER_NODE_H
