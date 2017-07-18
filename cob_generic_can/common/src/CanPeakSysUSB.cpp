/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


//#define __DEBUG__

#include <cob_generic_can/CanPeakSysUSB.h>
#include <stdlib.h>
#include <cerrno>
#include <cstring>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
//-----------------------------------------------
CANPeakSysUSB::CANPeakSysUSB(const char* device, int baudrate)
{
        m_bInitialized = false;

        p_cDevice = device;
        m_iBaudrateVal = baudrate;
}

CANPeakSysUSB::CANPeakSysUSB(const char* cIniFile)
{
        m_bInitialized = false;

        // read IniFile
        m_IniFile.SetFileName(cIniFile, "CanPeakSysUSB.cpp");

        init();
}

//-----------------------------------------------
CANPeakSysUSB::~CANPeakSysUSB()
{
        if (m_bInitialized)
        {
                CAN_Close(m_handle);
        }
}

//-----------------------------------------------
bool CANPeakSysUSB::init_ret()
{
        bool ret = true;

        // init() - part
        m_handle = LINUX_CAN_Open(p_cDevice, O_RDWR);

        if (! m_handle)
        {
                std::cout << "Cannot open CAN on USB: " << strerror(errno) << std::endl;
                ret = false;
        }
        else
        {
                ret = initCAN();
        }

        return ret;
}

//-----------------------------------------------
void CANPeakSysUSB::init()
{
        std::string sCanDevice;

        if( m_IniFile.GetKeyString( "TypeCan", "DevicePath", &sCanDevice, false) != 0) {
                sCanDevice = "/dev/pcan32";
        } else std::cout << "CAN-device path read from ini-File: " << sCanDevice << std::endl;

        //m_handle = LINUX_CAN_Open("/dev/pcan32", O_RDWR | O_NONBLOCK);
        m_handle = LINUX_CAN_Open(sCanDevice.c_str(), O_RDWR);

        if (! m_handle)
        {
                // Fatal error
                std::cout << "Cannot open CAN on USB: " << strerror(errno) << std::endl;
                sleep(3);
                exit(0);
        }

        m_iBaudrateVal = 0;
        m_IniFile.GetKeyInt( "CanCtrl", "BaudrateVal", &m_iBaudrateVal, true);

        initCAN();
}

//-------------------------------------------
bool CANPeakSysUSB::transmitMsg(CanMsg CMsg, bool bBlocking)
{
        TPCANMsg TPCMsg;
        bool bRet = true;

        if (m_bInitialized == false) return false;

        // copy CMsg to TPCmsg
        TPCMsg.LEN = CMsg.getLength();
        TPCMsg.ID = CMsg.getID();
        TPCMsg.MSGTYPE = CMsg.getType();
        for(int i=0; i<8; i++)
                TPCMsg.DATA[i] = CMsg.getAt(i);

        //TODO Hier stürtzt die Base ab.. verwende libpcan.h pcan.h um Fehler auszulesen, diagnostizieren, ausgeben und CAN_INIT erneut aufzurufen = neustart can-hardware.

        int iRet;
        //iRet = CAN_Write(m_handle, &TPCMsg);
        iRet = LINUX_CAN_Write_Timeout(m_handle, &TPCMsg, 25); //Timeout in micrsoseconds

        if(iRet != CAN_ERR_OK) {
#ifdef __DEBUG__
                std::cout << "CANPeakSysUSB::transmitMsg An error occured while sending..." << iRet << std::endl;
                outputDetailedStatus();
#endif
                bRet = false;
        }

#ifdef __DEBUG__
        //is this necessary? try iRet==CAN_Status(m_handle) ?
        iRet = CAN_Status(m_handle);

        if(iRet < 0)
        {
                std::cout <<  "CANPeakSysUSB::transmitMsg, system error: " << iRet << std::endl;
                bRet = false;
        } else if((iRet & CAN_ERR_BUSOFF) != 0) {
                std::cout <<  "CANPeakSysUSB::transmitMsg, BUSOFF detected" << std::endl;
                //Try to restart CAN-Device
                std::cout <<  "Trying to re-init Hardware..." << std::endl;
                bRet = initCAN();

        } else if((iRet & CAN_ERR_ANYBUSERR) != 0) {
                std::cout <<  "CANPeakSysUSB::transmitMsg, ANYBUSERR" << std::endl;

        } else if( (iRet & (~CAN_ERR_QRCVEMPTY)) != 0) {
                std::cout << "CANPeakSysUSB::transmitMsg, CAN_STATUS: " << iRet << std::endl;
                bRet = false;
        }
#endif

        return bRet;
}

//-------------------------------------------
bool CANPeakSysUSB::receiveMsg(CanMsg* pCMsg)
{
        TPCANRdMsg TPCMsg;
        TPCMsg.Msg.LEN = 8;
        TPCMsg.Msg.MSGTYPE = 0;
        TPCMsg.Msg.ID = 0;

        int iRet = CAN_ERR_OK;

        bool bRet = false;

        if (m_bInitialized == false) return false;

        iRet = LINUX_CAN_Read_Timeout(m_handle, &TPCMsg, 0);

        if (iRet == CAN_ERR_OK)
        {
                pCMsg->setID(TPCMsg.Msg.ID);
                pCMsg->setLength(TPCMsg.Msg.LEN);
                pCMsg->set(TPCMsg.Msg.DATA[0], TPCMsg.Msg.DATA[1], TPCMsg.Msg.DATA[2], TPCMsg.Msg.DATA[3],
                        TPCMsg.Msg.DATA[4], TPCMsg.Msg.DATA[5], TPCMsg.Msg.DATA[6], TPCMsg.Msg.DATA[7]);
                bRet = true;
        }
        else if( (iRet & (~CAN_ERR_QRCVEMPTY)) != 0) //no"empty-queue"-status
        {
                        std::cout << "CANPeakSysUSB::receiveMsg, CAN_STATUS: " << iRet << std::endl;
                        pCMsg->set(0, 0, 0, 0, 0, 0, 0, 0);
        }

        //catch status messages, these could be further processed in overlying software to identify and handle CAN errors
        if( TPCMsg.Msg.MSGTYPE == MSGTYPE_STATUS ) {
                std::cout << "CANPeakSysUSB::receiveMsg, status message catched:\nData is (CAN_ERROR_...) " << TPCMsg.Msg.DATA[3] << std::endl;
                pCMsg->set(0, 0, 0, 0, 0, 0, 0, 0);
        }

        return bRet;
}

//-------------------------------------------
bool CANPeakSysUSB::receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry)
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
                std::cout << "CANPeakSysUSB::receiveMsgRetry, errorcode= " << nGetLastError() << std::endl;
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
bool CANPeakSysUSB::receiveMsgTimeout(CanMsg* pCMsg, int nMicroSecTimeout)
{
    int iRet = CAN_ERR_OK;

    TPCANRdMsg TPCMsg;
    TPCMsg.Msg.LEN = 8;
    TPCMsg.Msg.MSGTYPE = 0;
    TPCMsg.Msg.ID = 0;

    if (m_bInitialized == false) return false;

    bool bRet = true;

    iRet = LINUX_CAN_Read_Timeout(m_handle, &TPCMsg, nMicroSecTimeout);

    // eval return value
    if(iRet != CAN_ERR_OK)
    {
	std::cout << "CANPeakSysUSB::receiveMsgTimeout, errorcode= " << nGetLastError() << std::endl;
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

bool CANPeakSysUSB::initCAN() {
        int ret = CAN_ERR_OK;
        bool bRet = true;

        switch(m_iBaudrateVal)
        {
        case CANITFBAUD_1M:
                ret = CAN_Init(m_handle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
                break;
        case CANITFBAUD_500K:
                ret = CAN_Init(m_handle, CAN_BAUD_500K, CAN_INIT_TYPE_ST);
                break;
        case CANITFBAUD_250K:
                ret = CAN_Init(m_handle, CAN_BAUD_250K, CAN_INIT_TYPE_ST);
                break;
        case CANITFBAUD_125K:
                ret = CAN_Init(m_handle, CAN_BAUD_125K, CAN_INIT_TYPE_ST);
                break;
        case CANITFBAUD_50K:
                ret = CAN_Init(m_handle, CAN_BAUD_50K, CAN_INIT_TYPE_ST);
                break;
        case CANITFBAUD_20K:
                ret = CAN_Init(m_handle, CAN_BAUD_20K, CAN_INIT_TYPE_ST);
                break;
        case CANITFBAUD_10K:
                ret = CAN_Init(m_handle, CAN_BAUD_10K, CAN_INIT_TYPE_ST);
                break;
        }

        if(ret)
        {
                std::cout << "CANPeakSysUSB::CANPeakSysUSB(), error in init" << std::endl;
                m_bInitialized = false;
                bRet = false;
        }
        else
        {
                std::cout << "CANPeakSysUSB::CanpeakSys(), init ok" << std::endl;
                m_bInitialized = true;
                bRet = true;
        }

        return bRet;
}

void CANPeakSysUSB::outputDetailedStatus() {
        TPDIAG diag;

        LINUX_CAN_Statistics(m_handle, &diag);

        std::cout << "*************************\n"
                        << "*** Detailed status output of CANPeakSys\n"
                        << "*************************"
                        << "\nIRQ-Level:     " << diag.wIrqLevel
                        << "\nNo reads:      " << diag.dwReadCounter
                        << "\nNo writes:     " << diag.dwWriteCounter
                        << "\nNo interrupts: " << diag.dwIRQcounter
                        << "\nNo errors:     " << diag.dwErrorCounter
                        << "\nError flag:    " << diag.wErrorFlag
                        << "\nLast error:    " << diag.nLastError
                        << std::endl;
}
