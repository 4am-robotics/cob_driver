//-----------------------------------------------
// Neobotix: Linux-Version
// www.neobotix.de
// Copyright (c) 2003. All rights reserved.

// author: Oliver Barth, Winfried Baum, Jens Kubacki
//-----------------------------------------------
//#include "stdafx.h"
#include <cob_forcetorque/CanESD.h>


//-----------------------------------------------
CanESD::CanESD(const char* cIniFile, bool bObjectMode)
{
	m_bObjectMode = bObjectMode;
	m_bIsTXError = false;
	//m_IniFile.SetFileName(cIniFile, "CanESD.cpp");
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
	//~CanItf();
}


//-----------------------------------------------
void CanESD::initIntern()
{	
	int ret=0;
/* Does this make any sense as the handle is opened somewhere later?!
	//m_Handle = 9;
	
	//m_Mutex.lock();

	//ret=canClose(m_Handle);

	if(ret == NTCAN_SUCCESS)
	{
		std::cout << "Can close ok ret = " << ret std::endl;
	}

	CAN_IF_STATUS *status;

	ret=canStatus(m_Handle,status);

	if(ret == NTCAN_SUCCESS)
	{
		std::cout << "Get status from can card ok ret = " << ret << std::endl;
	}
*/	
	ret = 0;
	int iCanNet = 0;
	//m_IniFile.GetKeyInt( "CanCtrl", "NetESD", &iCanNet, true);
	
	int iBaudrateVal = NTCAN_BAUD_250;
	//m_IniFile.GetKeyInt( "CanCtrl", "BaudrateVal", &iBaudrateVal, true);

	std::cout << "Initializing CAN network with id =" << iCanNet << ", baudrate=" << iBaudrateVal << std::endl;

	int iRet;
	if( m_bObjectMode )
		//iRet = canOpen(iCanNet, NTCAN_MODE_OBJECT, 10000, 10000, 0, 0, &m_Handle);
		iRet = canOpen(iCanNet, NTCAN_MODE_OBJECT, 10000, 10000, 1000, 0, &m_Handle);
	else
		//iRet = canOpen(iCanNet, 0, 10000, 10000, 0, 0, &m_Handle);
		iRet = canOpen(iCanNet, 0, 10000, 10000, 1000, 0, &m_Handle);
	sleep(0.3);

	if(iRet == NTCAN_SUCCESS)
		std::cout << "CanESD::CanESD(), init ok" << std::endl;
	else
		std::cout << "error in CANESD::receiveMsg: " << GetErrorStr(iRet) << std::endl;

/*	ret=canClose(m_Handle);
	if(ret == NTCAN_SUCCESS)
	{
		std::cout << "Can close ok ret = " << ret << std::endl;
		if( m_bObjectMode )
			iRet = canOpen(iCanNet, NTCAN_MODE_OBJECT, 10000, 10000, 0, 0, &m_Handle);
		else
			iRet = canOpen(iCanNet, 0, 10000, 10000, 0, 0, &m_Handle);
			Sleep(300);

		if(iRet == NTCAN_SUCCESS)
			std::cout << "CanESD::CanESD(), init ok" << ret << std::endl;
		else
			std::cout << "error in CANESD::receiveMsg: " << GetErrorStr(iRet) << std::endl;
	}
*/
	iRet = canSetBaudrate(m_Handle, iBaudrateVal);
	if(iRet == NTCAN_SUCCESS)
		std::cout << "CanESD::CanESD(), canSetBaudrate ok" << std::endl;
	else
		std::cout << "error in CANESD::receiveMsg: " << GetErrorStr(iRet) << std::endl;
	sleep(0.3);

	long lArg;
	iRet = canIoctl(m_Handle, NTCAN_IOCTL_FLUSH_RX_FIFO, NULL);
	//if( iRet == NTCAN_SUCCESS )
	//	std::cout << (int)(lArg) << " messages in CAN receive queue" << std::endl;

	// MMB/24.02.2006: Add all 11-bit identifiers as there is no loss in performance.
	for( int i=0; i<=0x7FF; ++i ) {
		iRet = canIdAdd( m_Handle, i );
		if(iRet != NTCAN_SUCCESS)
			std::cout << "error in CANESD::receiveMsg: " << GetErrorStr(iRet) << std::endl;
	}


	/*
	iRet = canIdAddGroup(m_Handle, 0x98);
	
	// for COB Arm
	iRet = canIdAdd(m_Handle, 0x100);
	iRet = canIdAdd(m_Handle, 0x101);
	iRet = canIdAdd(m_Handle, 0x200);
	iRet = canIdAdd(m_Handle, 0x201);
	iRet = canIdAdd(m_Handle, 0x300);
	iRet = canIdAdd(m_Handle, 0x301);
	iRet = canIdAdd(m_Handle, 0x400);
	iRet = canIdAdd(m_Handle, 0x401);
	iRet = canIdAdd(m_Handle, 0x500);
	iRet = canIdAdd(m_Handle, 0x501);
	iRet = canIdAdd(m_Handle, 0x600);
	iRet = canIdAdd(m_Handle, 0x601);
	
	// for CAN-Open Harmonica
	iRet = canIdAdd(m_Handle, 0x283);
	iRet = canIdAdd(m_Handle, 0x282);
	iRet = canIdAdd(m_Handle, 0x583);
	iRet = canIdAdd(m_Handle, 0x582);
	iRet = canIdAdd(m_Handle, 0x603);
	iRet = canIdAdd(m_Handle, 0x602);

	// for CAN-Open Harmonica and Cello Mimro
	iRet = canIdAdd(m_Handle, 0x18a);
	iRet = canIdAdd(m_Handle, 0x28a);
	iRet = canIdAdd(m_Handle, 0x194);
	iRet = canIdAdd(m_Handle, 0x294);
	iRet = canIdAdd(m_Handle, 0x19e);
	iRet = canIdAdd(m_Handle, 0x29e);
	
	// for RCS5000 arm
	iRet = canIdAdd(m_Handle, 0x111);
	iRet = canIdAdd(m_Handle, 0x112);
	iRet = canIdAdd(m_Handle, 0x113);
	iRet = canIdAdd(m_Handle, 0x114);
	iRet = canIdAdd(m_Handle, 0x115);
	iRet = canIdAdd(m_Handle, 0x116);
	iRet = canIdAdd(m_Handle, 0x788);
	iRet = canIdAdd(m_Handle, 0x789);
	iRet = canIdAdd(m_Handle, 0x78A);
	iRet = canIdAdd(m_Handle, 0x78B);
	iRet = canIdAdd(m_Handle, 0x78C);
	iRet = canIdAdd(m_Handle, 0x78D);
	iRet = canIdAdd(m_Handle, 0x78E);
	iRet = canIdAdd(m_Handle, 0x78F);

	iRet = canIdAddGroup(m_Handle, 0x00);
	iRet = canIdAddGroup(m_Handle, 0x08);
	iRet = canIdAddGroup(m_Handle, 0x10);
	iRet = canIdAddGroup(m_Handle, 0x18);
	iRet = canIdAddGroup(m_Handle, 0x20);
	iRet = canIdAddGroup(m_Handle, 0x28);
	iRet = canIdAddGroup(m_Handle, 0x30);
	iRet = canIdAddGroup(m_Handle, 0x38);
	iRet = canIdAddGroup(m_Handle, 0x40);
	iRet = canIdAddGroup(m_Handle, 0x48);
	iRet = canIdAddGroup(m_Handle, 0x50);
	iRet = canIdAddGroup(m_Handle, 0x58);
	iRet = canIdAddGroup(m_Handle, 0x60);
	iRet = canIdAddGroup(m_Handle, 0x68);
	iRet = canIdAddGroup(m_Handle, 0x98);

	// for Mimro drive ctrl
	iRet = canIdAdd(m_Handle, 0x123);
	*/

/*
	for(int i=0; i<2047; i++)
	{
		if (iRet != NTCAN_SUCCESS)
		{
			TRACE("canIdAdd: %x Error \n", i);
		}
	}
*/
	sleep(0.3);

	m_LastID = -1;

	//m_Mutex.unlock();
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
	{
		NTCANMsg.data[i] = CMsg.getAt(i);
		//std::cout << "Msg: "std::cout << CMsg.getAt(i) << std::endl;
	}
	
	int ret;
	//long len;
	int32_t len;
	bool bRet = true;
	
	len = 1;

	//m_Mutex.lock();
	if (bBlocking)
		ret = canWrite(m_Handle, &NTCANMsg, &len,+ NULL);
	else
		ret = canSend(m_Handle, &NTCANMsg, &len);

	//m_Mutex.unlock();

	if( ret != NTCAN_SUCCESS)
	{
		std::cout << "error in CANESD::transmitMsg: " << GetErrorStr(ret) << std::endl;
		bRet = false;
	}

	m_LastID = (int)NTCANMsg.data[0];

	//readEvent();

	//ret auswerten
	m_bIsTXError = !bRet;
	return bRet;
}

//-----------------------------------------------
bool CanESD::receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry)
{
	int id = pCMsg->m_iID;
	CMSG NTCANMsg;
	NTCANMsg.len = 8;
	
	//long len;
	int32_t len;
	int i, ret;
	bool bRet = true;
	
	i=0;
	
	len = 1;
	//NTCANMsg.id = pCMsg->m_iID;
	//ret = canTake(m_Handle, &NTCANMsg, &len);
	/**/
	do
	{
		len = 1;
		//m_Mutex.lock();
		ret = canTake(m_Handle, &NTCANMsg, &len);
		//m_Mutex.unlock();
		//if( NTCANMsg.id == id ) break;
		i++;
		sleep(0.01);
	}
	//while((len != 1) && (i < iNrOfRetry));
	//while((len != 1) && (i < iNrOfRetry));
	while((len == 0) && (i < iNrOfRetry));
	/**/
	
	//if((i == iNrOfRetry) || (ret != NTCAN_SUCCESS ))
	if(i == iNrOfRetry)
	//if( (ret != NTCAN_SUCCESS) || (len & 0x10) || (len == 0) )
	{
		if( ret != NTCAN_SUCCESS )
			std::cout << "error in CANESD::receiveMsgRetry: " << GetErrorStr(ret) << std::endl;
		//std::cout << "PCan:ReceiveMessage error, msg id= " << m_LastID << std::endl;
		//std::cout.unsetf(std::ios::dec);
		//std::cout.setf(std::ios::hex);
		//std::cout << "PCan:ReceiveMessage error, msg id= " << pCMsg->m_iID << std::endl;
		//std::cout.unsetf(std::ios::hex);
		//std::cout.setf(std::ios::dec);
		bRet = false;
	}
	else
	{
		pCMsg->m_iID = NTCANMsg.id;
		pCMsg->m_iLen = NTCANMsg.len;
		pCMsg->set(NTCANMsg.data[0], NTCANMsg.data[1], NTCANMsg.data[2], NTCANMsg.data[3],
			NTCANMsg.data[4], NTCANMsg.data[5], NTCANMsg.data[6], NTCANMsg.data[7]);
	}
		
	//readEvent();

	return bRet;
}

//-----------------------------------------------
bool CanESD::receiveMsg(CanMsg* pCMsg)
{
	CMSG NTCANMsg;
	NTCANMsg.len = 8;

	int ret;
	//long len;
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

	//m_Mutex.lock();
	ret = canRead(m_Handle, &NTCANMsg, &len, NULL);
	//m_Mutex.unlock();

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
	
//			std::cout << "len=" << len << "; ret = " << GetErrorStr(ret) << std::endl;
			bRet = false;
		}
	} else {
		//std::cout << "ID=" << NTCANMsg.id << "; len=" << (int)len << std::endl;
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

	//readEvent();
	/*
	DWORD status;
	long lArg;
	status = canIoctl(m_Handle, NTCAN_IOCTL_GET_RX_MSG_COUNT, &lArg);
	if( status == NTCAN_SUCCESS )
		std::cout << (int)(lArg) << " messages in CAN receive queue" << std::endl;
	*/
	return bRet;
}


/*!
    \fn CanESD::canIdAddGroup(m_Handle, int id, int number)
 */
/**
 * Add a group of CAN identifier to the handle, so it can be received.
 * The identifiers are generated by inverting the id and adding each value between 0 and 7
 * This is used for generating the answer commands by the RCS5000.
 * @param handle The handle to add the identifiers to.
 * @param id The command id sent to the RCS5000.
 * @return NTCAN_SUCESS if ok, or an error code.
 */
int CanESD::canIdAddGroup(HANDLE handle, int id)
{
	int result = NTCAN_SUCCESS;
	int i = 0;
	int iRes = 0;
	int cmd_id = invert(id);
	
	//m_Mutex.lock();
	for( i=0; i<8; ++i) {
		iRes = canIdAdd(m_Handle, cmd_id+i);
		//std::cout << "Adding CAN ID " << cmd_id+i << std::endl;
		if( iRes != NTCAN_SUCCESS ) {
			std::cout << "Adding CAN ID " << cmd_id+i << " failed with errorcode: " << iRes << " " << GetErrorStr(iRes) << std::endl;
			result = iRes;
		}
	}
	//m_Mutex.unlock();

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

	//m_Mutex.lock();
	ret = canReadEvent(m_Handle, &evmsg, NULL);
	//m_Mutex.unlock();
	
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

