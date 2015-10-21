#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "socketcan_interface/socketcan.h"
#include "socketcan_interface/threading.h"
#include "socketcan_interface/string.h"
#include <vector>
#include "cob_bms_driver/cob_bms_driver.h"

std::vector<std::string> paramater_ids_list1_;
std::vector<std::string> paramater_ids_list2_;


void pollCallback (std::string& response_string) {
	
	ROS_INFO_STREAM("this is response: " << response_string);
	
}



int main(int argc, char **argv) {
	
	//hardcoded parameter lists 
	paramater_ids_list1_.push_back("0101"); paramater_ids_list1_.push_back("0102"); paramater_ids_list1_.push_back("0106");
	
	paramater_ids_list2_.push_back("0115"); paramater_ids_list2_.push_back("0116"); paramater_ids_list2_.push_back("0117"); 
	paramater_ids_list2_.push_back("0118"); paramater_ids_list2_.push_back("0119"); paramater_ids_list2_.push_back("011A");
	paramater_ids_list2_.push_back("011B");
	
	BmsDriver bms_driver;
	
	if (bms_driver.initializeDriver() == false) {
		ROS_ERROR_STREAM("bms_driver initialization failed");
		return 1;
	}
				
	ros::init(argc, argv, "bms_driver_node");
	ros::NodeHandle n;
	//ros::Publisher canbus_pub = n.advertise<std_msgs::String>("canbus", 1000);
	
	size_t i = 0;
	size_t j = 0;

	//ros::Rate loop_rate(20);
	
	while (ros::ok())
    {
		
		if (i>=paramater_ids_list1_.size()) i=0;
		if (j>=paramater_ids_list2_.size()) j=0;

		ROS_INFO_STREAM("reading paramater_ids_list1_ at: " <<i);
		ROS_INFO_STREAM("reading paramater_ids_list2_ at: " <<j);
		
		bms_driver.pollBmsforParameters(paramater_ids_list1_.at(i), paramater_ids_list2_.at(j), pollCallback);
		
		++i;
		++j;	

		/*for(int i=0; i <10; ++i){  // send 10 messages, one per second
        		driver_.send(can::toframe("200#01010115")); // cansend syntax
       			
   		*/
  
		ros::spinOnce();

		//loop_rate.sleep();


    }
    
    return 0;	

}
