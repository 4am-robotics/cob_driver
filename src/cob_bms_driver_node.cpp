#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "socketcan_interface/socketcan.h"
#include "socketcan_interface/threading.h"
#include "socketcan_interface/string.h"

void handleFrames(const can::Frame &f){
    // handle all frames
    LOG("got: " << can::tostring(f, true)); // lowercase: true
}

/*void handleFrames(const can::Frame &f) {
	//handle specific frame
	LOG("123? " << can::tostring(f, true)); // lowercase: true
}*/
	
	
int main(int argc, char **argv) {
	
	ROS_INFO ("this is a test message");
	
	can::ThreadedSocketCANInterface driver;
	if(!driver.init("can0", false)) return 1; // read own messages: false
	
	can::CommInterface::FrameListener::Ptr all_frames = driver.createMsgListener(handleFrames);
	//can::CommInterface::FrameListener::Ptr one_frame = driver.createMsgListener(can::MsgHeader(0x123), handleFrame123); // handle only frames with CAN-ID 0x123
	
	ROS_INFO ("this is a test message2");
	
	ros::init(argc, argv, "akku_driver_node");
	ros::NodeHandle n;
	//ros::Publisher canbus_pub = n.advertise<std_msgs::String>("canbus", 1000);
	
	while (ros::ok())
    {
		ROS_INFO ("this is a test message LOOP");
		
		for(int i=0; i <10; ++i) {  // send 10 messages, one per second
			driver.send(can::toframe("200#01010105")); // cansend syntax
			boost::this_thread::sleep_for(boost::chrono::seconds(1));
		}
  
		ros::spinOnce();
    }
    
    driver.shutdown();
    return 0;	

}
