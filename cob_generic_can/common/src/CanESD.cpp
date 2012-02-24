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

// author: Oliver Barth, Marco Beesk
//-----------------------------------------------
#include <cob_generic_can/CanESD.h>

//-----------------------------------------------
CanESD::CanESD(int iIniBaudRate, int iNrNet)
{
	//initial values
	int net=1; /* logical net number (here: 0) */
	uint32_t mode=0; /* mode used for canOpen() */
	int32_t txqueuesize=100; /* size of transmit queue */
	int32_t rxqueuesize=100; /* size of receive queue */
	int32_t txtimeout=0; /* timeout for transmit operations in ms */
	int32_t rxtimeout=0; /* timeout for receive operations in ms */
	NTCAN_RESULT retvalue; /* return values of NTCAN API calls */
	//uint32_t baud=2; /* configured CAN baudrate (here: 500 kBit/s.) */
	CMSG cmsg[8]; /* can message buffer */
	int rtr=0; /* rtr bit */
	int32_t len; /* # of CAN messages */

	retvalue = canOpen(iNrNet, mode, txqueuesize, rxqueuesize, txtimeout, rxtimeout, &m_Handle);
	if (retvalue != NTCAN_SUCCESS)
	{
		LOGINFO("canOpen() failed: " << getErrorMessage(retvalue));
	}
	Sleep(300);
	retvalue = canSetBaudrate(m_Handle, getBaudRate(iIniBaudRate));
	if (retvalue != 0)
	{
		LOGINFO("canSetBaudrate() failed: " << getErrorMessage(retvalue)); 
		canClose(m_Handle);
	}
	else 
	{
		LOGINFO("Baudrate set to " << getBaudRate(iIniBaudRate));	
	}
	Sleep(300);
	
	//set Amtek Arm IDs
	int i;
	for(i=1; i < 2048; i++)
	{
		retvalue = canIdAdd(m_Handle, i);
		if (retvalue != NTCAN_SUCCESS)
		{
			LOGERROR("canIdAdd() failed: " << getErrorMessage(retvalue)); 
		}
	}

	Sleep(300);

	m_LastID = -1;
}
//-----------------------------------------------
CanESD::~CanESD()
{
	int retvalue;

	retvalue = canClose(m_Handle);
	if(retvalue != NTCAN_SUCCESS)
	{
		LOGERROR("can not close handle");
	}
}

//-----------------------------------------------
void CanESD::init(int t)
{
}
//-----------------------------------------------
bool CanESD::transmitMsg(CanMsg &CMsg)
{
  	CMSG NTCANMsg;
	NTCANMsg.id = CMsg.m_iID;
	NTCANMsg.len = CMsg.m_iLen;

	for(int i=0; i<8; i++)
		NTCANMsg.data[i] = CMsg.getAt(i);
	
	NTCAN_RESULT ret;
	int32_t len;
	bool bRet = true;
	len = 1;
	//ret = canWrite(m_Handle, canmsg, &len, NULL);
	ret = canSend(m_Handle, &NTCANMsg, &len);

	if( ret != NTCAN_SUCCESS)
	{
		LOGERROR("canSend() failed: " << getErrorMessage(ret));
		bRet = false;
	}

	m_LastID = (int)NTCANMsg.data[0];
	//ret auswerten
	return bRet;
}

//-----------------------------------------------
bool CanESD::receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry)
{
	CMSG NTCANMsg;
	NTCANMsg.len = 8;
	
	int32_t len;
	int i, ret;
	bool bRet = true;

	i=0;
	do
	{
		len = 1;
		ret = canTake(m_Handle, &NTCANMsg, &len);
		i++;
		Sleep(10);
	}
	while((len != 1) && (i < iNrOfRetry));

	if(i == iNrOfRetry)
	{
		std::cout << "PCan:ReceiveMessage error, msg code= " << m_LastID << std::endl;
		bRet = false;
	}
	else
	{
		pCMsg->m_iID = NTCANMsg.id;
		pCMsg->set(NTCANMsg.data[0], NTCANMsg.data[1], NTCANMsg.data[2], NTCANMsg.data[3],
			NTCANMsg.data[4], NTCANMsg.data[5], NTCANMsg.data[6], NTCANMsg.data[7]);
	}
	return bRet;
}

//-----------------------------------------------
bool CanESD::receiveMsg(CanMsg* pCMsg)
{
  
	CMSG NTCANMsg;
	NTCANMsg.len = 8;

	int ret;
	int32_t len;
	bool bRet = true;
	len = 1;
	//ret = canRead(m_Handle, &NTCANMsg, &len, NULL);
	ret = canTake(m_Handle, &NTCANMsg, &len);
	
	if (len == 1)
	{
		// message received
		pCMsg->m_iID = NTCANMsg.id;
		pCMsg->set(NTCANMsg.data[0], NTCANMsg.data[1], NTCANMsg.data[2], NTCANMsg.data[3],
			NTCANMsg.data[4], NTCANMsg.data[5], NTCANMsg.data[6], NTCANMsg.data[7]);
		bRet = true;
	}
	else
	{
		// no message
		if( ret != NTCAN_SUCCESS)
		{
			// error
			LOGERROR("canTake() failed: " << getErrorMessage(ret));
			
		}

		pCMsg->m_iID = NTCANMsg.id;
		pCMsg->set(0,0,0,0,0,0,0,0);

		bRet = false;
	}
	return bRet;
}

//-----------------------------------------------
bool CanESD::emMessageError()
{
  return false;
}

//-----------------------------------------------
std::string CanESD::getErrorMessage(NTCAN_RESULT res)
{
	if(res==NTCAN_SUCCESS)
		return ("");
	
	if(res==NTCAN_RX_TIMEOUT)
		return ("RX_TIMEOUT\n");	
	if(res==NTCAN_TX_ERROR)
		return ("NTCAN_TX_ERROR\n");	
	if(res==NTCAN_CONTR_OFF_BUS)
		return ("NTCAN_CONTR_OFF_BUS\n");	
	if(res==NTCAN_CONTR_BUSY)
		return ("NTCAN_CONTR_BUSY\n");	
	if(res==NTCAN_CONTR_WARN)
		return ("NTCAN_CONTR_WARN\n");	
	if(res==NTCAN_NO_ID_ENABLED)
		return ("NTCAN_NO_ID_ENABLED\n");	
	if(res==NTCAN_ID_ALREADY_ENABLED)
		return ("NTCAN_ID_ALREADY_ENABLED\n");	
	if(res==NTCAN_ID_NOT_ENABLED)
		return ("NTCAN_ID_NOT_ENABLED\n");	
	if(res==NTCAN_NO_ID_ENABLED)
		return ("NTCAN_NO_ID_ENABLED\n");	
	if(res==NTCAN_INVALID_FIRMWARE)
		return ("NTCAN_INVALID_FIRMWARE\n");
	if(res==NTCAN_MESSAGE_LOST)
		return ("NTCAN_MESSAGE_LOST\n");	
	if(res==NTCAN_INVALID_HARDWARE)
		return ("NTCAN_INVALID_HARDWARE\n");	
	if(res==NTCAN_PENDING_WRITE)
		return ("NTCAN_PENDING_WRITE\n");	
	if(res==NTCAN_PENDING_READ)
		return ("NTCAN_PENDING_READ\n");	
	if(res==NTCAN_INVALID_DRIVER)
		return ("NTCAN_INVALID_DRIVER\n");	
	
	return ("unknown error ");
}

//-----------------------------------------------
int CanESD::getBaudRate(int iIniBaud)
{
	if(iIniBaud==0)
		return NTCAN_BAUD_1000;
	if(iIniBaud==1)
		return NTCAN_BAUD_500;
	if(iIniBaud==2)
		return NTCAN_BAUD_250;
	if(iIniBaud==3)
		return NTCAN_BAUD_125;
	if(iIniBaud==4)
		return NTCAN_BAUD_100;
	if(iIniBaud==5)
		return NTCAN_BAUD_50;
	if(iIniBaud==6)
		return NTCAN_BAUD_20;
	if(iIniBaud==7)
		return NTCAN_BAUD_10;
	else
		LOGINFO("Baudrate " << iIniBaud << " not supported");
		return -1;

}

