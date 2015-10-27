#ifndef COB_BMS_DRIVER_H
#define COB_BMS_DRIVER_H

#include <vector>
#include <string>
//#include <sstream>
#include "socketcan_interface/socketcan.h"
#include "socketcan_interface/threading.h"
#include "socketcan_interface/string.h"
#include "ros/ros.h"

#include <socketcan_interface/dummy.h>	// -- REMOVE LATER

class BmsDriver 
{
	public:
	
		//TODO
		unsigned int bms_id_;

		//function that polls all batteries (i.e. at CAN ID: 0x200) for two parameters at a time
		bool pollBmsforParameters(const char first_parameter_id, const char second_parameter_id /*, void (*callback)(std::string&)*/);
	
		//function to return reference to driver instance
		//can::ThreadedSocketCANInterface& getDriverRef(); UNCOMMENT LATER
		can::DummyInterface& getDriverRef() ;

		//handler for all frames
		void handleFrames(const can::Frame &f);

		//void handleAccuCurrent(const can::Frame &f);
	
		//function to initialize driver with device can0 and register handleFrames() function for handling all frames, returns false if initialization fails
		bool initializeDriver();
		
		//BmsDriver(); UNCOMMENT LATER
		//~BmsDriver();
		
		BmsDriver() : driver_(true) { }	// REMOVE LATER

	private:
		//can::ThreadedSocketCANInterface driver_; -- UNCOMMENT LATER
		can::DummyInterface driver_;
		
		//void (*handleFrameCallback)(std::string&);
		can::CommInterface::FrameListener::Ptr frame_listener_;
		//can::CommInterface::FrameListener::Ptr accu_current_listener_;
};


		
#endif
