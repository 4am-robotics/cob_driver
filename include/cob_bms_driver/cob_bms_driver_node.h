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

		//ROS parameters
		std::map<char, std::vector<BmsParameter> > config_map_;	//holds all the information that is provided in the config.yaml file
		int poll_period_for_two_ids_in_ms_;
		std::string can_device_;								
		int bms_id_to_poll_;

		//lists that contain diagnostics ids that are to be polled, each field/element in a diagnostic id is a BmsParameter
		std::vector<char> polling_list1_;
		std::vector<char> polling_list2_;
		std::vector<char>::iterator polling_list1_it_;
		std::vector<char>::iterator polling_list2_it_;

		//interface to send frames to and read frames from BMS
		can::ThreadedSocketCANInterface socketcan_interface_;

		//callback function to handle CAN frames from BMS
		can::CommInterface::FrameListener::Ptr frame_listener_;

		//holds diagnostics data received from BMS
		diagnostic_updater::DiagnosticStatusWrapper stat_;
		
		//function to get parameters from parameter server
		void getParams();

		//function to interpret the diagnostics XmlRpcValue and save data in config_map_
		void loadConfigMap(XmlRpc::XmlRpcValue diagnostics, std::vector<std::string> topics);

		//function to create a publisher for each Topic that is listed in the configuration file
		void createPublishersFor(std::vector<std::string> topics);

		//helper function to evaluate poll period from given poll frequency
		void evaluatePollPeriodFrom(int poll_frequency);

		//function that goes through config_map_ and fills polling_list1_ and polling_list2_. If topics are found on ROS Parameter Server, they are kept in list1 otherwise, all parameter id are divided between both lists.
		void loadPollingLists();
	
		//function that polls BMS for given ids (NOTE: bms_id_to_poll_ is used here!)
		//also, this function sleeps for time given by poll_period_for_two_ids_in_ms_ to ensure BMS is polled at the desired frequency 
		void pollBmsForIds(const char first_id, const char second_id);
		
		//callback function to handle all types of frames received from BMS
		void handleFrames(const can::Frame &f);

	public:
	
		ros::NodeHandle nh_;
		ros::NodeHandle nh_priv_;

		//responsible for updating the diagnostics data
		diagnostic_updater::Updater updater_;

		//publishes the BMS parameter data to their respective topics (name of the topic is same as the name of the BMS parameter)
		std::map<std::string, ros::Publisher> bms_diagnostics_publishers_;
		
		CobBmsDriverNode();
		~CobBmsDriverNode();

		//initlializes SocketCAN interface, calls functions to save all ROS parameters to their respective variables in this class, loads polling lists and sets up (diagnostics) updater_
		bool prepare();

		//cycles through polling lists and sends 2 ids at a time (one from each list) to the BMS
		void pollNextInLists();
		
		//updates the diagnostics data with the new data received from BMS
		void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);	
};


#endif //COB_BMS_DRIVER_NODE_H
