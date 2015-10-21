#ifndef COB_BMS_DRIVER_H
#define COB_BMS_DRIVER_H

#include <vector>
#include <string>
//#include <sstream>
#include "socketcan_interface/socketcan.h"
#include "socketcan_interface/threading.h"
#include "socketcan_interface/string.h"


class BmsDriver {
	
	
	public:
	
		//function that polls all batteries (i.e. at CAN ID: 0x200) for two parameters at a time
		bool pollBmsforParameters(const std::string first_parameter_id, const std::string second_parameter_id);
	
		//function to return reference to driver instance
		can::ThreadedSocketCANInterface& getDriverRef();

		//handler for all frames
		void handleFrames(const can::Frame &f);
	
		//function to initialize driver with device can0 and register handleFrames() function for handling all frames, returns false if initialization fails
		bool initializeDriver();
		
		BmsDriver();
		~BmsDriver();
	
	private:
		can::ThreadedSocketCANInterface driver_;		
};


		
#endif
