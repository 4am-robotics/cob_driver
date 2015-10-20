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
		bool pollBmsforParameters(const std::string data_req_id1, const std::string data_req_id2);
	
		//function to return reference to driver instance
		can::ThreadedSocketCANInterface& getDriverRef();

		//handler for all frames
		void handleFrames(const can::Frame &f);
		
		void setHandlerForAllFrames();
		
		BmsDriver();
		~BmsDriver();
	
	private:
		can::ThreadedSocketCANInterface driver;		
};


		
#endif
