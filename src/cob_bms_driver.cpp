#include "cob_bms_driver/cob_bms_driver.h"

BmsDriver::BmsDriver() {
	
}

BmsDriver::~BmsDriver()  {
	driver.shutdown();
}


//function that polls all batteries (i.e. at CAN ID: 0x200) for two parameters at a time
bool BmsDriver::pollBmsforParameters(const std::string data_req_id1, const std::string data_req_id2) {
	
	driver.send(can::toframe("200#"+data_req_id1+data_req_id2)); 
	boost::this_thread::sleep_for(boost::chrono::seconds(1));
	
	return true;

}

//handler for all frames
void BmsDriver::handleFrames(const can::Frame &f){
    LOG("got: " << can::tostring(f, true)); 
}

can::ThreadedSocketCANInterface& BmsDriver::getDriverRef() {
	return driver;
}

void BmsDriver::setHandlerForAllFrames() {
	
	can::CommInterface::FrameListener::Ptr all_frames = driver.createMsgListener(can::CommInterface::FrameDelegate(this, &BmsDriver::handleFrames));
	
	//emcy_listener_ = interface->createMsgListener( emcy_id.header(), can::CommInterface::FrameDelegate(this, &EMCYHandler::handleEMCY));	
	
}




/*void handleFrames(const can::Frame &f) {
	//handle specific frame
	LOG("123? " << can::tostring(f, true)); // lowercase: true
}*/
