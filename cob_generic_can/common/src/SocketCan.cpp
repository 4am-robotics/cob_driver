/****************************************************************

 ****************************************************************/

//#define __DEBUG__

#include <cob_generic_can/SocketCan.h>
#include <stdlib.h>
#include <cerrno>
#include <cstring>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <class_loader/class_loader.h>
#include <socketcan_interface/socketcan.h>
#include <boost/unordered_set.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/make_shared.hpp>

//-----------------------------------------------
SocketCan::SocketCan(const char* device, int baudrate) : SocketCANInterface()
{
        m_bInitialized = false;
	recived = false;

        p_cDevice = device;
	
	CommInterface::FrameListener::Ptr frame_reciver = g_driver->createMsgListener(SocketCan::recive_frame);
	StateInterface::StateListener::Ptr error_reciver = g_driver->createStateListener(SocketCan::recive_error);
}

SocketCan::SocketCan(const char* cIniFile)
{
        m_bInitialized = false;

        // read IniFile
        m_IniFile.SetFileName(cIniFile, "SocktCan.cpp");
 
        init();
}

//-----------------------------------------------
SocketCan::~SocketCan()
{
        if (m_bInitialized)
        {
               shutdown(); // welche aufgabe muss erledigt werden wenn dekonstrulieren
        }
}

//-----------------------------------------------
bool SocketCan::init_ret()
{
        bool ret = true;

        // init() - part
        if (!init(p_cDevice, false))
        {
                std::cout << "Cannot open SocketCan: " << getState() << std::endl;
                ret = false;
        }
        else
        {
                ret = initCAN();
		run();
        }

        return ret;
}

//-----------------------------------------------
void SocketCan::init()
{
        std::string sCanDevice;
	

        if( m_IniFile.GetKeyString( "TypeCan", "DevicePath", &sCanDevice, false) != 0) {
                sCanDevice = "/dev/pcan32";
        } else std::cout << "CAN-device path read from ini-File: " << sCanDevice << std::endl;

        //m_handle = LINUX_CAN_Open("/dev/pcan32", O_RDWR | O_NONBLOCK);
        //m_handle = LINUX_CAN_Open(sCanDevice.c_str(), O_RDWR);
	m_handle = g_driver->init(sCanDevice.c_str(), false);

        if (! m_handle)
        {
                // Fatal error
                std::cout << "Cannot open CAN on USB: " << strerror(errno) << std::endl;
                sleep(3);
                exit(0);
        }

        m_iBaudrateVal = 0;
        m_IniFile.GetKeyInt( "CanCtrl", "BaudrateVal", &m_iBaudrateVal, true);

}


//-------------------------------------------
bool SocketCan::transmitMsg(CanMsg CMsg, bool bBlocking)
{
	can::Frame message(CMsg.getID(), CMsg.getLength());
	for(int i=0; i<CMsg.getLength(); i++)
	    message.data[i] = CMsg.getAt(i);
	
	return send(message);
}

//-------------------------------------------
bool SocketCan::receiveMsg(CanMsg* pCMsg)
{
       if (m_bInitialized == false) return false;

       bool bRet = false;
       
       if (recived) {
	  
	  pCMsg->setID(recived_frame.id);
	  pCMsg->setLength(recived_frame.l);
	  pCMsg->set(recived_frame.data[0], recived_frame.data[1], recived_frame.data[2], recived_frame.data[3],
                        recived_frame.data[4], recived_frame.data[5], recived_frame.data[6], recived_frame.data[7]);
	  bRet = true;
	}
	if (!recived){
	  std::cout << "no message recived: " << std::endl;
	}
	return bRet;
}

//-------------------------------------------
bool SocketCan::receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry)
{
        int i, iRet;

        TPCANRdMsg TPCMsg;
        TPCMsg.Msg.LEN = 8;
        TPCMsg.Msg.MSGTYPE = 0;
        TPCMsg.Msg.ID = 0;

        if (m_bInitialized == false) return false;

        // wait until msg in buffer
        bool bRet = true;
        iRet = CAN_ERR_OK;
        i=0;
        do
        {
                iRet = LINUX_CAN_Read_Timeout(m_handle, &TPCMsg, 0);

                if(iRet == CAN_ERR_OK)
                        break;

                i++;
                usleep(10000);
        }
        while(i < iNrOfRetry);

        // eval return value
        if(iRet != CAN_ERR_OK)
        {
                std::cout << "SocketCan::receiveMsgRetry, errorcode= " << nGetLastError() << std::endl;
                pCMsg->set(0, 0, 0, 0, 0, 0, 0, 0);
                bRet = false;
        }
        else
        {
                pCMsg->setID(TPCMsg.Msg.ID);
                pCMsg->setLength(TPCMsg.Msg.LEN);
                pCMsg->set(TPCMsg.Msg.DATA[0], TPCMsg.Msg.DATA[1], TPCMsg.Msg.DATA[2], TPCMsg.Msg.DATA[3],
                        TPCMsg.Msg.DATA[4], TPCMsg.Msg.DATA[5], TPCMsg.Msg.DATA[6], TPCMsg.Msg.DATA[7]);
        }

        return bRet;
}

//-------------------------------------------
bool SocketCan::receiveMsgTimeout(CanMsg* pCMsg, int nSecTimeout)
{
    int iRet = CAN_ERR_OK;

    TPCANRdMsg TPCMsg;
    TPCMsg.Msg.LEN = 8;
    TPCMsg.Msg.MSGTYPE = 0;
    TPCMsg.Msg.ID = 0;

    if (m_bInitialized == false) return false;

    bool bRet = true;

    iRet = LINUX_CAN_Read_Timeout(m_handle, &TPCMsg, nSecTimeout);

    // eval return value
    if(iRet != CAN_ERR_OK)
    {
	std::cout << "SocketCan::receiveMsgTimeout, errorcode= " << nGetLastError() << std::endl;
	pCMsg->set(0, 0, 0, 0, 0, 0, 0, 0);
	bRet = false;
    }
    else
    {
	pCMsg->setID(TPCMsg.Msg.ID);
	pCMsg->setLength(TPCMsg.Msg.LEN);
	pCMsg->set(TPCMsg.Msg.DATA[0], TPCMsg.Msg.DATA[1], TPCMsg.Msg.DATA[2], TPCMsg.Msg.DATA[3],
		    TPCMsg.Msg.DATA[4], TPCMsg.Msg.DATA[5], TPCMsg.Msg.DATA[6], TPCMsg.Msg.DATA[7]);
    }

    return bRet;
}

bool SocketCan::initCAN() {
	m_bInitialized = true;
        bool bRet = true;
        return true;
}

void SocketCan::recive_frame(const Frame& frame){
  
    if(frame.is_error){
        std::cout << "E " << std::hex << frame.id << std::dec;
    }else if(frame.is_extended){
        std::cout << "e " << std::hex << frame.id << std::dec;
    }else{
        std::cout << "s " << std::hex << frame.id << std::dec;
    }
    
    std::cout << "\t";
    
    if(frame.is_rtr){
        std::cout << "r";
    }else{
        std::cout << (int) frame.dlc << std::hex;
        
        for(int i=0; i < frame.dlc; ++i){
            std::cout << std::hex << " " << (int) frame.data[i];
        }
    }
    
    std::cout << std::dec << std::endl;
    recived_frame = frame;
    recived = true;
}

void SocketCan::recive_error(const State & state){
     std::string err;
     std::cout << "ERROR: state=" << std::endl;
    //g_driver->translateError(s.internal_error,err);
    //std::cout << "ERROR: state=" << s.driver_state << " internal_error=" << s.internal_error << "('" << err << "') asio: " << s.error_code << std::endl;
}
