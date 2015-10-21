#include "cob_bms_driver/cob_bms_driver.h"

BmsDriver::BmsDriver() {
	
}

BmsDriver::~BmsDriver()  {
	driver_.shutdown();
}

can::ThreadedSocketCANInterface& BmsDriver::getDriverRef() {
	return driver_;
}


//function that polls all batteries (i.e. at CAN ID: 0x200) for two parameters at a time, TODO: check that parameter ids are valid
bool BmsDriver::pollBmsforParameters(const std::string first_parameter_id, const std::string second_parameter_id) {
	
	std::string msg = "200#"+first_parameter_id+second_parameter_id;
	driver_.send(can::toframe(msg)); 
	//ROS_INFO_STREAM("sending message: " << msg);
	boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
	
	return true;

}

//function to initialize driver with device can0 and register handleFrames() function for handling all frames, returns false if initialization fails
bool BmsDriver::initializeDriver() {
	
	if(!driver_.init("can0", false)) {
		return false;	
	}
	can::CommInterface::FrameListener::Ptr all_frames = driver_.createMsgListener(can::CommInterface::FrameDelegate(this, &BmsDriver::handleFrames));
	//emcy_listener_ = interface->createMsgListener( emcy_id.header(), can::CommInterface::FrameDelegate(this, &EMCYHandler::handleEMCY));	
	return true;
}


//handler for all frames
void BmsDriver::handleFrames(const can::Frame &f){
	
	ROS_INFO_STREAM ("got :" << can::tostring(f, true));
		
    LOG("got: " << can::tostring(f, true)); 
}

//can::CommInterface::FrameListener::Ptr one_frame = driver_.createMsgListener(can::MsgHeader(0x123), handleFrame123); // handle only frames with CAN-ID 0x123

/*void handleFrames(const can::Frame &f) {
	//handle specific frame
	LOG("123? " << can::tostring(f, true)); // lowercase: true
}*/

