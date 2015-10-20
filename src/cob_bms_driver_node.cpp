#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "socketcan_interface/socketcan.h"
#include "socketcan_interface/threading.h"
#include "socketcan_interface/string.h"


can::ThreadedSocketCANInterface driver_;

void handleFrames(const can::Frame &f){
    // handle all frames
    LOG("got: " << can::tostring(f, true
}

/*void handleFrames(const can::Frame &f) {
	//handle specific frame
	LOG("123? " << can::tostring(f, true)); // lowercase: true
}*/

//function that polls all batteries (i.e. at CAN ID: 0x200) for two parameters at a time
bool pollBMSforCANids(const std::string first_parameter_id, const std::string second_parameter_id) {
	
	strd::string msg = "200#"+first_parameter_id+second_parameter_id;
	driver_.send(can::toframe(msg)); 
	ROS_INFO_STREAM("sending message: " << msg);
	boost::this_thread::sleep_for(boost::chrono::milliseconds(25));
	
	return true;

}	
	
int main(int argc, char **argv) {
	
	if(!driver_.init("can0", false)) return 1; // read own messages: false
	
	can::CommInterface::FrameListener::Ptr all_frames = driver_.createMsgListener(handleFrames);
	//can::CommInterface::FrameListener::Ptr one_frame = driver_.createMsgListener(can::MsgHeader(0x123), handleFrame123); // handle only frames with CAN-ID 0x123
		
	ros::init(argc, argv, "bms_driver_node");
	ros::NodeHandle n;
	//ros::Publisher canbus_pub = n.advertise<std_msgs::String>("canbus", 1000);
	
	while (ros::ok())
    {
	
		pollBMSforCANids("0101", "0105");
  
		ros::spinOnce();
    }
    
    driver_.shutdown();
    return 0;	

}
