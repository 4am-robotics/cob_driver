#include "cob_bms_driver/cob_bms_driver.h"

#include <stdint.h>
#include <endian.h>

template<int N> void big_endian_to_host(const void* in, void* out);
template<> void big_endian_to_host<1>(const void* in, void* out){ *(uint8_t*)out = *(uint8_t*)in;}
template<> void big_endian_to_host<2>(const void* in, void* out){ *(uint16_t*)out = be16toh(*(uint16_t*)in);}
template<> void big_endian_to_host<4>(const void* in, void* out){ *(uint32_t*)out = be32toh(*(uint32_t*)in);}
template<> void big_endian_to_host<8>(const void* in, void* out){ *(uint64_t*)out = be64toh(*(uint64_t*)in);}

template<typename T> T read_value(const can::Frame &f, uint8_t offset){
	T res;
	big_endian_to_host<sizeof(T)>(&f.data[offset], &res);
	return res;
}

BmsDriver::BmsDriver() {
	
}

BmsDriver::~BmsDriver()  {
	driver_.shutdown();
}

can::ThreadedSocketCANInterface& BmsDriver::getDriverRef() {
	return driver_;
}


//function that polls all batteries (i.e. at CAN ID: 0x200) for two parameters at a time, TODO: check that parameter ids are valid
bool BmsDriver::pollBmsforParameters(const std::string first_parameter_id, const std::string second_parameter_id, void (*callback)(std::string&)){
	
	handleFrameCallback = callback;
	
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
	frame_listener_  = driver_.createMsgListener(can::CommInterface::FrameDelegate(this, &BmsDriver::handleFrames));
	accu_current_listener_  = driver_.createMsgListener(can::MsgHeader(0x102), can::CommInterface::FrameDelegate(this, &BmsDriver::handleAccuCurrent));
	//emcy_listener_ = interface->createMsgListener( emcy_id.header(), can::CommInterface::FrameDelegate(this, &EMCYHandler::handleEMCY));	
	return true;
}


void BmsDriver::handleAccuCurrent(const can::Frame &f){
	
	if(f.dlc >= 2){
		LOG("accu_current: " << read_value<int16_t>(f,0) * 0.01); 
	}
}

//handler for all frames
void BmsDriver::handleFrames(const can::Frame &f){
	
	std::string msg = "got: " + can::tostring(f, true);
	
	handleFrameCallback(msg);
	
	LOG(msg);
	
	//ROS_INFO_STREAM("got :" << can::tostring(f, true));	
    //LOG("got: " << can::tostring(f, true)); 
}

//can::CommInterface::FrameListener::Ptr one_frame = driver_.createMsgListener(can::MsgHeader(0x123), handleFrame123); // handle only frames with CAN-ID 0x123

/*void handleFrames(const can::Frame &f) {
	//handle specific frame
	LOG("123? " << can::tostring(f, true)); // lowercase: true
}*/

