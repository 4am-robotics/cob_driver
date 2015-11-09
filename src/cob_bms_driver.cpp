#include "cob_bms_driver/cob_bms_driver.h"

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

/*void BmsDriver::setConfigMap(const std::vector<std::map<int, BmsParameter> >& config_map_ref) 
{
	config_map_ref_ = config_map_ref;	
}*/

can::ThreadedSocketCANInterface& BmsDriver::getDriverRef() {
	return driver_;
}


//function that polls all batteries (i.e. at CAN ID: 0x200) for two parameters at a time, TODO: check that parameter ids are valid
bool BmsDriver::pollBmsforParameters(const char first_parameter_id, const char second_parameter_id/*, void (*callback)(std::string&)*/){
		
	can::Frame f(can::Header(bms_id_,false,false,false),4);
	f.data[0] = 0x01;
	f.data[1] = first_parameter_id;
	f.data[2] = 0x01;
	f.data[3] = second_parameter_id;
	
	driver_.send(f);
	
	//ROS_INFO_STREAM("sending message: " << msg);
	boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
	
	return true;

}

//function to initialize driver with device can0 and register handleFrames() function for handling all frames, returns false if initialization fails
bool BmsDriver::initializeDriver() {
	
	BmsDriver::bms_id_ = 0x200;	
	
	if(!driver_.init("can0", false)) {
		return false;	
	}
	
	//create listeners for CAN frames
	frame_listener_  = driver_.createMsgListener(can::CommInterface::FrameDelegate(this, &BmsDriver::handleFrames));
	
	
	//accu_current_listener_  = driver_.createMsgListener(can::MsgHeader(0x102), can::CommInterface::FrameDelegate(this, &BmsDriver::handleAccuCurrent));
	//emcy_listener_ = interface->createMsgListener( emcy_id.header(), can::CommInterface::FrameDelegate(this, &EMCYHandler::handleEMCY));	
	return true;
}


/*void BmsDriver::handleAccuCurrent(const can::Frame &f){
	
	if(f.dlc >= 2){
		LOG("accu_current: " << read_value<int16_t>(f,0) * 0.01); 
	}
}*/

//TODO: extend this to all frame types!!
//handler for all frames
void BmsDriver::handleFrames(const can::Frame &f){
	
	std::string msg = "handling: " + can::tostring(f, true);
	LOG(msg);
	
	/*if(f.dlc >= 2) {
				double accu_current = read_value<int16_t>(f,0) * 0.01;	
				LOG("accu_current: " << accu_current); 
				//accu_pub.publish(accu_current); --TODO			
				//diagnostic_updater::DiagnosticStatusWrapper &stat	--TODO
				//stat.add("accu_current", accu_current);
	}*/
			
	switch(f.id) {
		
		//config_map_ref_.find 
		
		case 0x102:
			if(f.dlc >= 2) {
				double accu_current = read_value<int16_t>(f,0) * 0.01;	
				LOG("accu_current: " << accu_current); 
				//accu_pub.publish(accu_current); --TODO			
				//diagnostic_updater::DiagnosticStatusWrapper &stat	--TODO
				//stat.add("accu_current", accu_current);
			}
			break;
			
			
		case 0x103:
			if(f.dlc >= 2) {
				double accu_pack_voltage = read_value<int16_t>(f,0) * 0.01;	
				LOG("accu_pack_voltage: " << accu_pack_voltage); 
				//accu_pub.publish(accu_current); --TODO			
				//diagnostic_updater::DiagnosticStatusWrapper &stat	--TODO
				//stat.add("accu_current", accu_current);
			}
			break;
		
			
	};

	//handleFrameCallback(msg);
}

//can::CommInterface::FrameListener::Ptr one_frame = driver_.createMsgListener(can::MsgHeader(0x123), handleFrame123); // handle only frames with CAN-ID 0x123

/*void handleFrames(const can::Frame &f) {
	//handle specific frame
	LOG("123? " << can::tostring(f, true)); // lowercase: true
}*/

