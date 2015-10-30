#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "socketcan_interface/socketcan.h"
#include "socketcan_interface/threading.h"
#include "socketcan_interface/string.h"
#include <vector>
#include "cob_bms_driver/cob_bms_driver.h"

std::vector<char> paramater_ids_list1_;
std::vector<char> paramater_ids_list2_;

void loadParameters(const& ros::NodeHandle nh) {
	    
    std::vector<XmlRpc::XmlRpcValue> xdiagnostics, xdiagnostic_elements, xfields;
    std::map<std::string, XmlRpc::XmlRpcValue> xpair;
    
    std::vector<DiagnosticClass> diagnostics;
    DiagnosticClass diagnostic_object;
    
    nh.getParam("/diagnostics", xdiagnostics);
    ROS_ASSERT(xdiagnostics.getType() == XmlRpc::XmlRpcValue::ValueArray);
    
    for (int32_t i = 0; i < xdiagnostics.size(); ++i) {

		//get diagnostic_elements, i.e. id and fields
		ROS_ASSERT(xdiagnostics[i].getType() == XmlRpc::XmlRpcValue::ValueArray);
		xdiagnostic_elements = static_cast<std::vector<XmlRpc::XmlRpcValue>(xdiagnostics[i]);
				
		//get id 
		ROS_ASSERT(xdiagnostic_elements[0].getType() == XmlRpc::XmlRpcValue::TypeStruct);
		ROS_ASSERT(xdiagnostic_elements[0].second.getType() == XmlRpc::XmlRpcValue::TypeString);
		diagnostic_object.id  = static_cast<std::string>(xdiagnostic_elements[0].second);	//save value of id from first diagnostic element //TODO: convert string to hex later
			
		//get fields
		ROS_ASSERT(xdiagnostic_elements[1].getType() == XmlRpc::XmlRpcValue::ValueStruct);
		ROS_ASSERT(xdiagnostic_elements[1].second.getType() == XmlRpc::XmlRpcValue::ValueArray);
		xfields  = static_cast<std::vector<XmlRpcValue> xdiagnostic_elements[1].second;
		
		//extract data from fields
		std::vector<DiagnosticClass::Field> fields(xfields.size(), DiagnosticClass::Field::Empty);
		for (int32_t j = 0; j < xfields.size(); ++j) {
			
			//name
			ROS_ASSERT(xfields[0].getType() == XmlRpc::XmlRpcValue::TypeStruct);
			ROS_ASSERT(xfields[0].second.getType() == XmlRpc::XmlRpcValue::TypeString);
			fields[j].name = static_cast<std::string> xfields[0].second;
			
			//offset
			ROS_ASSERT(xfields[1].getType() == XmlRpc::XmlRpcValue::TypeStruct);
			ROS_ASSERT(xfields[1].second.getType() == XmlRpc::XmlRpcValue::TypeInt);	//TODO: this should be unsigned. but no such option, check if this will not be a problem
			fields[j].offset = static_cast<unsigned int> xfields[1].second;
			
			//length
			ROS_ASSERT(xfields[2].getType() == XmlRpc::XmlRpcValue::TypeStruct);
			ROS_ASSERT(xfields[2].second.getType() == XmlRpc::XmlRpcValue::TypeInt);	//TODO: this should be unsigned. but no such option, check if this will not be a problem
			fields[j].length = static_cast<unsigned int> xfields[2].second;
			
			//sign flag
			ROS_ASSERT(xfields[3].getType() == XmlRpc::XmlRpcValue::TypeStruct);
			ROS_ASSERT(xfields[2].second.getType() == XmlRpc::XmlRpcValue::TypeBoolean);
			fields[j].sign_flag = static_cast<bool> xfields[3].second;
			
			//factor
			ROS_ASSERT(xfields[4].getType() == XmlRpc::XmlRpcValue::TypeStruct);
			ROS_ASSERT(xfields[2].second.getType() == XmlRpc::XmlRpcValue::TypeInt);
			fields[j].factor = static_cast<unsigned int> xfields[4].second;				//TODO: this should be unsigned. but no such option, check if this will not be a problem
			
			//unit
			ROS_ASSERT(xfields[5].getType() == XmlRpc::XmlRpcValue::TypeStruct);
			ROS_ASSERT(xfields[5].second.getType() == XmlRpc::XmlRpcValue::TypeString);
			fields[j].unit = static_cast<std::string> xfields[5].second;

	}
	
	
	
}

/*void pollCallback (std::string& response_string) {
	
	ROS_INFO_STREAM("this is response: " << response_string);
	
}*/

int main(int argc, char **argv) {
	
	//hardcoded parameter lists 
	paramater_ids_list1_.push_back(0x02); paramater_ids_list1_.push_back(0x03); paramater_ids_list1_.push_back(0x06);
	
	paramater_ids_list2_.push_back(0x15); /*paramater_ids_list2_.push_back("0116"); paramater_ids_list2_.push_back("0117"); 
	paramater_ids_list2_.push_back("0118"); paramater_ids_list2_.push_back("0119"); paramater_ids_list2_.push_back("011A");
	paramater_ids_list2_.push_back("011B");*/
	
	BmsDriver bms_driver;
	
	/*if (bms_driver.initializeDriver() == false) {
		ROS_ERROR_STREAM("bms_driver initialization failed");
		return 1;
	}*/
				
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
		
		bms_driver.pollBmsforParameters(paramater_ids_list1_.at(i), paramater_ids_list2_.at(j));
		
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
