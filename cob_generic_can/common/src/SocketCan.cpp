/*****************************************************************************
 * Copyright 2015 Intelligent Industrial Robotics (IIROB) Group,
 * Institute for Anthropomatics and Robotics (IAR) -
 * Intelligent Process Control and Robotics (IPR),
 * Karlsruhe Institute of Technology

 * This package is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************/

#include <cob_generic_can/SocketCan.h>
#include <stdlib.h>
#include <cerrno>
#include <cstring>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

SocketCan::SocketCan(const char* device, int baudrate)
{
        m_bInitialized = false;
        recived = false;

        p_cDevice = device;
        can::SocketCANInterface m_handle;
}

SocketCan::SocketCan(const char* device)
{
        m_bInitialized = false;
        recived = false;

        p_cDevice = device;
        can::SocketCANInterface m_handle;
}

//-----------------------------------------------
SocketCan::~SocketCan()
{
        if (m_bInitialized)
        {
               m_handle.shutdown(); // welche aufgabe muss erledigt werden wenn dekonstrulieren
        }
}

//-----------------------------------------------
bool SocketCan::init_ret()
{
        bool ret = true;

        // init() - part
        if (!m_handle.init(p_cDevice, false))
        {
                print_error(m_handle.getState());
                ret = false;
        }
        else
        {
                ret = initCAN();
		m_handle.run();
        }

        return ret;
}

//-----------------------------------------------
void SocketCan::init()
{
        if (!init_ret())
        {
                sleep(3);
                exit(0);
        }
}


//-------------------------------------------
bool SocketCan::transmitMsg(CanMsg CMsg, bool bBlocking)
{
    can::Header header(CMsg.getID(), false, false, false);
	can::Frame message(header, CMsg.getLength());
	for(int i=0; i<CMsg.getLength(); i++)
	    message.data[i] = CMsg.getAt(i);

	return m_handle.send(message);
}

//-------------------------------------------
bool SocketCan::receiveMsg(CanMsg* pCMsg)
{
    if (m_bInitialized == false) return false;

    bool bRet = false;

    if (recived) {
        pCMsg->setID(recived_frame.id);
        pCMsg->setLength(recived_frame.dlc);
        pCMsg->set(recived_frame.data[0], recived_frame.data[1], recived_frame.data[2], recived_frame.data[3],
                        recived_frame.data[4], recived_frame.data[5], recived_frame.data[6], recived_frame.data[7]);
        bRet = true;
	}
	else {
	  std::cout << "No message recived: " << std::endl;
	}
	return bRet;
}

//-------------------------------------------
bool SocketCan::receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry)
{
    if (m_bInitialized == false) return false;

    // wait until msg in buffer
    bool bRet = false;
    int i=0;
    do
    {
        if (recived) {
            pCMsg->setID(recived_frame.id);
            pCMsg->setLength(recived_frame.dlc);
            pCMsg->set(recived_frame.data[0], recived_frame.data[1], recived_frame.data[2], recived_frame.data[3],
                            recived_frame.data[4], recived_frame.data[5], recived_frame.data[6], recived_frame.data[7]);
            bRet = true;
            break;
        }
        i++;
        usleep(10000);
    }
    while(i < iNrOfRetry);
    return bRet;
}

//-------------------------------------------
bool SocketCan::receiveMsgTimeout(CanMsg* pCMsg, int nSecTimeout)
{
    if (m_bInitialized == false) return false;

    // wait until msg in buffer
    bool bRet = false;

    usleep(nSecTimeout/1000);
    if (recived) {
        pCMsg->setID(recived_frame.id);
        pCMsg->setLength(recived_frame.dlc);
        pCMsg->set(recived_frame.data[0], recived_frame.data[1], recived_frame.data[2], recived_frame.data[3],
                        recived_frame.data[4], recived_frame.data[5], recived_frame.data[6], recived_frame.data[7]);
        bRet = true;
    }

    return bRet;
}

bool SocketCan::initCAN() {
	m_bInitialized = true;
        bool bRet = true;
        return true;
}

void SocketCan::recive_frame(const can::Frame& frame){

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

void SocketCan::print_error(const can::State & state){
     std::string err;
     std::cout << "ERROR: state=" << std::endl;
    m_handle.translateError(state.internal_error, err);
    std::cout << "ERROR: state=" << state.driver_state << " internal_error=" << state.internal_error << "('" << err << "') asio: " << state.error_code << std::endl;
}
