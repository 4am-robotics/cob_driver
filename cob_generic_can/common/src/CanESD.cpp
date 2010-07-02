/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_drivers
 * ROS package name: cob_generic_can
 * Description: This class implements the interface to an ESD can node
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: April 2010
 * ToDo: - Remove Mutex.h search for a Boost lib
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

// general includes

// Headers provided by other cob-packages
#include <cob_generic_can/CanESD.h>

// Headers provided by other cob-packages which should be avoided/removed


//-----------------------------------------------
CanESD::CanESD(const char* cIniFile, bool bObjectMode)
{
	m_bObjectMode = bObjectMode;
	m_bIsTXError = false;
	m_IniFile.SetFileName(cIniFile, "CanESD.cpp");
	initIntern();
}

//-----------------------------------------------
/**
 * Destructor.
 * Release the allocated resources.
 */
CanESD::~CanESD()
{
	std::cout << "Closing CAN handle" << std::endl;
	canClose(m_Handle);
}


//-----------------------------------------------
void CanESD::initIntern()
{	
	int ret=0;	
	ret = 0;
	int iCanNet = 1;
	m_IniFile.GetKeyInt( "CanCtrl", "NetESD", &iCanNet, true);
	
	int iBaudrateVal = 2;
	m_IniFile.GetKeyInt( "CanCtrl", "BaudrateVal", &iBaudrateVal, true);

	std::cout << "Initializing CAN network with id =" << iCanNet << ", baudrate=" << iBaudrateVal << std::endl;

	int iRet;
	if( m_bObjectMode )
		iRet = canOpen(iCanNet, NTCAN_MODE_OBJECT, 10000, 10000, 1000, 0, &m_Handle);
	else
		iRet = canOpen(iCanNet, 0, 10000, 10000, 1000, 0, &m_Handle);
	Sleep(300);

	if(iRet == NTCAN_SUCCESS)
		std::cout << "CanESD::CanESD(), init ok" << std::endl;
	else
		std::cout << "error in CANESD::receiveMsg: " << GetErrorStr(iRet) << std::endl;

	iRet = canSetBaudrate(m_Handle, iBaudrateVal);
	if(iRet == NTCAN_SUCCESS)
		std::cout << "CanESD::CanESD(), canSetBaudrate ok" << std::endl;
	else
		std::cout << "error in CANESD::receiveMsg: " << GetErrorStr(iRet) << std::endl;
	Sleep(300);

	long lArg;
	iRet = canIoctl(m_Handle, NTCAN_IOCTL_FLUSH_RX_FIFO, NULL);

	// MMB/24.02.2006: Add all 11-bit identifiers as there is no loss in performance.
	for( int i=0; i<=0x7FF; ++i ) {
		iRet = canIdAdd( m_Handle, i );
		if(iRet != NTCAN_SUCCESS)
			std::cout << "error in CANESD::receiveMsg: " << GetErrorStr(iRet) << std::endl;
	}


	Sleep(300);

	m_LastID = -1;
}

//-----------------------------------------------
/**
 * Transmit a message via the CAN bus.
 * Additionally, an error flag is set internally if the transmission does not succeed.
 * @param CMsg Structure containing the CAN message.
 * @return true on success, false on failure.
 */
bool CanESD::transmitMsg(CanMsg CMsg, bool bBlocking)
{
	CMSG NTCANMsg;
	NTCANMsg.id = CMsg.m_iID;
	NTCANMsg.len = CMsg.m_iLen;

	for(int i=0; i<8; i++)
		NTCANMsg.data[i] = CMsg.getAt(i);
	
	int ret;
	int32_t len;
	bool bRet = true;
	
	len = 1;

	if (bBlocking)
		ret = canWrite(m_Handle, &NTCANMsg, &len, NULL);
	else
		ret = canSend(m_Handle, &NTCANMsg, &len);

	if( ret != NTCAN_SUCCESS)
	{
		std::cout << "error in CANESD::transmitMsg: " << GetErrorStr(ret) << std::endl;
		bRet = false;
	}

	m_LastID = (int)NTCANMsg.data[0];

	m_bIsTXError = !bRet;
	return bRet;
}

//-----------------------------------------------
bool CanESD::receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry)
{
	int id = pCMsg->m_iID;
	CMSG NTCANMsg;
	NTCANMsg.len = 8;
	
	int32_t len;
	int i, ret;
	bool bRet = true;
	
	i=0;
	
	len = 1;

	do
	{
		len = 1;
		ret = canTake(m_Handle, &NTCANMsg, &len);
		i++;
		Sleep(10);
	}

	while((len == 0) && (i < iNrOfRetry));
	
	if(i == iNrOfRetry)
	{
		if( ret != NTCAN_SUCCESS )
			std::cout << "error in CANESD::receiveMsgRetry: " << GetErrorStr(ret) << std::endl;

		bRet = false;
	}
	else
	{
		pCMsg->m_iID = NTCANMsg.id;
		pCMsg->m_iLen = NTCANMsg.len;
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

	// Debug valgrind
	NTCANMsg.data[0] = 0;
	NTCANMsg.data[1] = 0;
	NTCANMsg.data[2] = 0;
	NTCANMsg.data[3] = 0;
	NTCANMsg.data[4] = 0;
	NTCANMsg.data[5] = 0;
	NTCANMsg.data[6] = 0;
	NTCANMsg.data[7] = 0;
	NTCANMsg.msg_lost = 0;
	NTCANMsg.id = 0;
	NTCANMsg.len = 0;

	pCMsg->set(0,0,0,0,0,0,0,0);

	
	if( !isObjectMode() ) {
		pCMsg->m_iID = 0;
	} else {
		NTCANMsg.id = pCMsg->m_iID;
	}

	ret = canTake(m_Handle, &NTCANMsg, &len);

	if( !isObjectMode() ) {
		if( (len == 1) && (ret == NTCAN_SUCCESS) )
		{
			// message received
			pCMsg->m_iID = NTCANMsg.id;
			pCMsg->m_iLen = NTCANMsg.len;
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
				std::cout << "error in CANESD::receiveMsg: " << GetErrorStr(ret) << std::endl;
			}
	
			pCMsg->m_iID = NTCANMsg.id;
			pCMsg->set(0,0,0,0,0,0,0,0);

			bRet = false;
		}
	} else {
		if( len == 16 ) {
			// No message was received yet.
			pCMsg->m_iID = NTCANMsg.id;
			pCMsg->set(0,0,0,0,0,0,0,0);
			bRet = false;
		} else {
			pCMsg->m_iID = NTCANMsg.id;
			pCMsg->m_iLen = NTCANMsg.len;
			pCMsg->set(NTCANMsg.data[0], NTCANMsg.data[1], NTCANMsg.data[2], NTCANMsg.data[3],
				   NTCANMsg.data[4], NTCANMsg.data[5], NTCANMsg.data[6], NTCANMsg.data[7]);
			bRet = true;
		}
	}
	
	if( NTCANMsg.msg_lost != 0 )
		std::cout << (int)(NTCANMsg.msg_lost) << " messages lost!" << std::endl;

	return bRet;
}


/**
 * Add a group of CAN identifier to the handle, so it can be received.
 * The identifiers are generated by inverting the id and adding each value between 0 and 7
 * This is used for generating the answer commands by the RCS5000.
 * @param handle The handle to add the identifiers to.
 * @param id The command id sent to the RCS5000.
 * @return NTCAN_SUCESS if ok, or an error code.
 */
int CanESD::canIdAddGroup(NTCAN_HANDLE handle, int id)
{
	int result = NTCAN_SUCCESS;
	int i = 0;
	int iRes = 0;
	int cmd_id = invert(id);

	for( i=0; i<8; ++i) {
		iRes = canIdAdd(m_Handle, cmd_id+i);

		if( iRes != NTCAN_SUCCESS ) {
			std::cout << "Adding CAN ID " << cmd_id+i << " failed with errorcode: " << iRes << " " << GetErrorStr(iRes) << std::endl;
			result = iRes;
		}
	}

	return result;
}

//-----------------------------------------------
std::string CanESD::GetErrorStr(int ntstatus) const
{
	switch (ntstatus)
	{
	case NTCAN_SUCCESS:			return "NTCAN_SUCCESS";
	case NTCAN_RX_TIMEOUT:			return "NTCAN_RX_TIMEOUT";
	case NTCAN_TX_TIMEOUT:			return "NTCAN_TX_TIMEOUT";
	case NTCAN_TX_ERROR:			return "NTCAN_TX_ERROR";
	case NTCAN_CONTR_OFF_BUS:		return "NTCAN_CONTR_OFF_BUS";
	case NTCAN_CONTR_BUSY:			return "NTCAN_CONTR_BUSY";
	case NTCAN_CONTR_WARN:			return "NTCAN_CONTR_WARN";
	case NTCAN_NO_ID_ENABLED:		return "NTCAN_NO_ID_ENABLED";
	case NTCAN_ID_ALREADY_ENABLED:		return "NTCAN_ID_ALREADY_ENABLED";
	case NTCAN_ID_NOT_ENABLED:		return "NTCAN_ID_NOT_ENABLED";

	case NTCAN_INVALID_FIRMWARE:		return "NTCAN_INVALID_FIRMWARE";
	case NTCAN_MESSAGE_LOST:		return "NTCAN_MESSAGE_LOST";
	case NTCAN_INVALID_HARDWARE:		return "NTCAN_INVALID_HARDWARE";

	case NTCAN_PENDING_WRITE:		return "NTCAN_PENDING_WRITE";
	case NTCAN_PENDING_READ:		return "NTCAN_PENDING_READ";
	case NTCAN_INVALID_DRIVER:		return "NTCAN_INVALID_DRIVER";

	case NTCAN_INVALID_PARAMETER:		return "NTCAN_INVALID_PARAMETER";
	case NTCAN_INVALID_HANDLE:		return "NTCAN_INVALID_HANDLE";
	case NTCAN_NET_NOT_FOUND:		return "NTCAN_NET_NOT_FOUND";
	case NTCAN_INSUFFICIENT_RESOURCES:	return "NTCAN_INSUFFICIENT_RESOURCES";
	
	case NTCAN_OPERATION_ABORTED:		return "NTCAN_OPERATION_ABORTED";
	}
	char msg[100];
	sprintf(msg, "unknown error code %d", ntstatus);
	return msg;
}

/**
 * Check if errors occured on the CAN bus.
 * @return 	- 0 if everthing is fine.
 *		- -1 if an error occured.
 *		- -3 if messages were lost.
 *		- -5 if a FIFO overflow occured.
 *		- -6 if the CAN controller is BUS OFF.
 *		- -7 if the CAN controller is WARN, i.e. error passive.
 */
int CanESD::readEvent()
{
	EVMSG evmsg;
	int iRet = 0;
	int ret;

	ret = canReadEvent(m_Handle, &evmsg, NULL);
	
	if(ret == NTCAN_SUCCESS)
	{
		if( (int)evmsg.evid == NTCAN_EV_CAN_ERROR ) {
			switch( evmsg.evdata.s[0] ) {
				case 0x00:
					iRet = 0;
					break;
				case 0xC0:
					iRet = -6;
					std::cout << "BUS OFF" << std::endl;
					break;
				case 0x40:
					iRet = -7;
					std::cout << "ERROR PASSIVE" << std::endl;
					break;
			}
			if( evmsg.evdata.s[3] != 0 ) {
				iRet = -3;
				std::cout << "Lost " << (int)evmsg.evdata.s[3] << " messages" << std::endl;
			} else if( evmsg.evdata.s[5] != 0 ) {
				iRet = -5;
				std::cout << "Lost " << (int)evmsg.evdata.s[5] << " messages from fifo" << std::endl;
			}
		}
	}
	return iRet;
}

