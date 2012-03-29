/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <cob_generic_can/CANPeakSysUSB.h>
#include <libpcan.h>

#include <cerrno>
#include <cstring>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


//-----------------------------------------------
CANPeakSysUSB::CANPeakSysUSB(int iBaudRate, std::string strDeviceName)
{
	m_bInitialized = false;
	m_handle = LINUX_CAN_Open(strDeviceName.c_str(), O_RDWR);

	if (! m_handle)
	{
		// Fatal error
		LOGERROR("can not open CANPeakSysUSB device : " << strerror(errno));
		usleep(3000);
		exit(0);
	}
OUTPUTINFO("baudrate: %i", iBaudRate);
	init(iBaudRate);
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
void CANPeakSysUSB::init(int iBaudRate)
{

	
	
	int ret = CAN_ERR_OK;
	switch(iBaudRate)
	{
	case 0:
		ret = CAN_Init(m_handle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
		break;

	case 1:
		ret = CAN_Init(m_handle, CAN_BAUD_500K, CAN_INIT_TYPE_ST);
		break;

	case 2:
		ret = CAN_Init(m_handle, CAN_BAUD_250K, CAN_INIT_TYPE_ST);
		break;

	default:
		ret = CAN_Init(m_handle, CAN_BAUD_500K, CAN_INIT_TYPE_ST);
	}

	if(ret)
	{
		LOGERROR("init()" );
	}
	else
	{
		LOGINFO("init ok" );
		m_bInitialized = true;
	}
}

//-------------------------------------------
bool CANPeakSysUSB::transmitMsg(CanMsg& CMsg)
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
		LOGERROR( "transmitMsg() : errorcode= " << nGetLastError() );
		bRet = false;
	}


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
		pCMsg->m_iID = TPCMsg.Msg.ID;
		pCMsg->set(TPCMsg.Msg.DATA[0], TPCMsg.Msg.DATA[1], TPCMsg.Msg.DATA[2], TPCMsg.Msg.DATA[3],
			TPCMsg.Msg.DATA[4], TPCMsg.Msg.DATA[5], TPCMsg.Msg.DATA[6], TPCMsg.Msg.DATA[7]);
		bRet = true;
	}
	else if (CAN_Status(m_handle) != CAN_ERR_QRCVEMPTY)
	{
		// Error (Don't output error of empty queue)
		LOGERROR( "receiveMsg() : errorcode= " << nGetLastError() );
		pCMsg->set(0, 0, 0, 0, 0, 0, 0, 0);
	}

	return bRet;
}

bool CANPeakSysUSB::emMessageError()
{
  return false;
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
		usleep(100);
	}
	while(i < iNrOfRetry);

	// eval return value
	if(iRet != CAN_ERR_OK)
	{
		LOGERROR("receiveMsgRetry() : errorcode= " << nGetLastError() );
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

