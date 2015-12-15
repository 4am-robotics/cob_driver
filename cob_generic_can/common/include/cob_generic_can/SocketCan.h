/****************************************************************
 
 ****************************************************************/

#ifndef CANPEAKSYSUSB_INCLUDEDEF_H
#define CANPEAKSYSUSB_INCLUDEDEF_H
//-----------------------------------------------
#include <cob_generic_can/CanItf.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cob_utilities/IniFile.h>
#include <socketcan_interface/socketcan.h>
//-----------------------------------------------

class SocketCan : public can::SocketCANInterface
{ 
public:
	// --------------- Interface
	SocketCan(const char* device, int baudrate);
	SocketCan(const char* cIniFile);
	~SocketCan();
	bool init_ret();
	void init();
	void destroy() {};
	bool transmitMsg(CanMsg CMsg, bool bBlocking = true);
	bool receiveMsg(CanMsg* pCMsg);
	bool receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry);
	bool receiveMsgTimeout(CanMsg* pCMsg, int nMicroSeconds);
	bool isObjectMode() { return false; }

private:
	// --------------- Types
	HANDLE m_handle;

	bool m_bInitialized;
	IniFile m_IniFile;
	bool m_bSimuEnabled;
	const char* p_cDevice;
	int m_iBaudrateVal;
	
	bool recived;
	can::Frame recived_frame;

	static const int c_iInterrupt;
	static const int c_iPort;

	bool initCAN();
	
	void recive_frame(const Frame & frame);
	void recive_error(const State & state);
};
//-----------------------------------------------
#endif


