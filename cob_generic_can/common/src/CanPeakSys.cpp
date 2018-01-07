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


#include <cob_generic_can/CanPeakSys.h>
#include <stdlib.h>
#include <cerrno>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
//-----------------------------------------------

const int CanPeakSys::c_iInterrupt = 7;
const int CanPeakSys::c_iPort = 0x378;

//-----------------------------------------------
CanPeakSys::CanPeakSys(const char* cIniFile)
{
	m_bInitialized = false;

	// read IniFile
	m_IniFile.SetFileName(cIniFile, "CanPeakSys.cpp");
	init();
}

//-----------------------------------------------
CanPeakSys::~CanPeakSys()
{
	if (m_bInitialized)
	{
		CAN_Close(m_handle);
	}
}

//-----------------------------------------------
bool CanPeakSys::init_ret()
{
	// Not implemented yet
	return false;
}

//-----------------------------------------------
void CanPeakSys::init()
{
 std::string sCanDevice;
 if( m_IniFile.GetKeyString( "TypeCan", "DevicePath", &sCanDevice, false) != 0) {
		sCanDevice = "/dev/pcan32";
	} else std::cout << "CAN-device path read from ini-File: " << sCanDevice << std::endl;
	m_handle = LINUX_CAN_Open(sCanDevice.c_str(), O_RDWR);


	if (! m_handle)
	{
		// Fatal error
		std::cout << "Cannot open CAN-dongle on parallel port: " << strerror(errno) << std::endl;
		sleep(3);
		exit(0);
	}


	int ret = CAN_ERR_OK;
	int iBaudrateVal = 0;
	m_IniFile.GetKeyInt( "CanCtrl", "BaudrateVal", &iBaudrateVal, true);

	switch(iBaudrateVal)
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
		std::cout << "CanPeakSys::CanPeakSys(), error in init" << std::endl;
	}
	else
	{
		std::cout << "CanPeakSys::CanpeakSys(), init ok" << std::endl;
		m_bInitialized = true;
	}
}

//-------------------------------------------
bool CanPeakSys::transmitMsg(CanMsg CMsg, bool bBlocking)
{
	TPCANMsg TPCMsg;
	bool bRet = true;

	if (m_bInitialized == false) return false;

	// copy CMsg to TPCmsg
	TPCMsg.LEN = CMsg.m_iLen;
	TPCMsg.ID = CMsg.m_iID;
	TPCMsg.MSGTYPE = CMsg.m_iType;
	for(int i=0; i<8; i++)
		TPCMsg.DATA[i] = CMsg.getAt(i);

	// write msg
	int iRet;
	iRet = CAN_Write(m_handle, &TPCMsg);
	iRet = CAN_Status(m_handle);

	if(iRet < 0)
	{
		std::cout << "CanPeakSys::transmitMsg, errorcode= " << nGetLastError() << std::endl;
		bRet = false;
	}


	return bRet;
}

//-------------------------------------------
bool CanPeakSys::receiveMsg(CanMsg* pCMsg)
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
		pCMsg->m_iID = TPCMsg.Msg.ID;
		pCMsg->set(TPCMsg.Msg.DATA[0], TPCMsg.Msg.DATA[1], TPCMsg.Msg.DATA[2], TPCMsg.Msg.DATA[3],
			TPCMsg.Msg.DATA[4], TPCMsg.Msg.DATA[5], TPCMsg.Msg.DATA[6], TPCMsg.Msg.DATA[7]);
		bRet = true;
	}
	else if (CAN_Status(m_handle) != CAN_ERR_QRCVEMPTY)
	{
		std::cout << "CanPeakSys::receiveMsg ERROR: iRet = " << iRet << std::endl;
		pCMsg->set(0, 0, 0, 0, 0, 0, 0, 0);
	}
	else
	{
		// make sure there's never an undefined state (even when can drivers fail)
		pCMsg->set(0, 0, 0, 0, 0, 0, 0, 0);
	}

	return bRet;
}

//-------------------------------------------
bool CanPeakSys::receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry)
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
		usleep(100000);
	}
	while(i < iNrOfRetry);

	// eval return value
	if(iRet != CAN_ERR_OK)
	{
		std::cout << "CanPeakSys::receiveMsgRetry: " << strerror(errno) << std::endl;
		pCMsg->set(0, 0, 0, 0, 0, 0, 0, 0);
		bRet = false;
	}
	else
	{
		pCMsg->m_iID = TPCMsg.Msg.ID;
		pCMsg->set(TPCMsg.Msg.DATA[0], TPCMsg.Msg.DATA[1], TPCMsg.Msg.DATA[2], TPCMsg.Msg.DATA[3],
			TPCMsg.Msg.DATA[4], TPCMsg.Msg.DATA[5], TPCMsg.Msg.DATA[6], TPCMsg.Msg.DATA[7]);
	}

	return bRet;
}

//-------------------------------------------
bool CanPeakSys::receiveMsgTimeout(CanMsg* pCMsg, int nMicroSecTimeout)
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
