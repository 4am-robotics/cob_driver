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
 * ROS stack name: cob_driver
 * ROS package name: cob_relayboard
 * Description: Class for communication with relayboard. The relayboard is mainly used for reading the Emergencystop and Laserscannerstop states.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Philipp Koehler
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: March 2010
 * ToDo:
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


#include <math.h>
#include <cob_relayboard/SerRelayBoard.h>

//-----------------------------------------------


#define NUM_BYTE_SEND 50 //Total amount of data sent to relayboard in one message

#define RS422_BAUDRATE 420000
#define RS422_RX_BUFFERSIZE 1024
#define RS422_TX_BUFFERSIZE 1024

#define RS422_TIMEOUT 0.025

#define NUM_BYTE_REC_MAX 120
#define NUM_BYTE_REC_HEADER 4 //Header bytes, which are used to identify the beginning of a new received message {0x02, 0x80, 0xD6, 0x02}
#define NUM_BYTE_REC_CHECKSUM 2 //checksum for message, that is built as the sum of all data bytes contained in the message
#define NUM_BYTE_REC 104 //Total amount of data bytes in a received message (from the relayboard)


//-----------------------------------------------
SerRelayBoard::SerRelayBoard(std::string ComPort)
{
	m_bComInit = false;
	m_sNumComPort = ComPort;

	m_iRelBoardBattVoltage = 0;
	m_iConfigRelayBoard = 0;
	m_iRelBoardKeyPad = 0xFFFF;
	m_iCmdRelayBoard = 0;
	m_iDigIn = 0;
}

//-----------------------------------------------
SerRelayBoard::~SerRelayBoard()
{
	m_SerIO.close();
}

//-----------------------------------------------
int SerRelayBoard::evalRxBuffer()
{
	static int siNoMsgCnt = 0;
	
	const int c_iNrBytesMin = NUM_BYTE_REC_HEADER + NUM_BYTE_REC + NUM_BYTE_REC_CHECKSUM;
	const int c_iSizeBuffer = 4096;

	int i;
	int errorFlag = NO_ERROR;
	int iNrBytesInQueue, iNrBytesRead, iDataStart;
	unsigned char cDat[c_iSizeBuffer];
	unsigned char cTest[4] = {0x02, 0x80, 0xD6, 0x02};

	if( !m_bComInit ) return NOT_INITIALIZED;

	//enough data in queue?
	iNrBytesInQueue = m_SerIO.getSizeRXQueue();
	if(iNrBytesInQueue < c_iNrBytesMin)
	{
		//there are too less bytes in queue
		siNoMsgCnt++;
		if(siNoMsgCnt > 29)
		{
			//std::cerr << "Relayboard: " << siNoMsgCnt << " cycles no msg received";
			siNoMsgCnt = 0;
			errorFlag = NO_MESSAGES;
		} else errorFlag = TOO_LESS_BYTES_IN_QUEUE;
		
		return errorFlag;
	}
	else
	{
		siNoMsgCnt = 0;
	}

	// search most recent data from back of queue
	iNrBytesRead = m_SerIO.readBlocking((char*)&cDat[0], iNrBytesInQueue);
	for(i = (iNrBytesRead - c_iNrBytesMin); i >= 0 ; i--)
	{
		//try to find start bytes
		if((cDat[i] == cTest[0]) && (cDat[i+1] == cTest[1]) && (cDat[i+2] == cTest[2]) && (cDat[i+3] == cTest[3]))
		{
			iDataStart = i + 4;

			// checksum ok?
			if( convRecMsgToData(&cDat[iDataStart]) )
			{
				return errorFlag;
			}
			else
			{
				//std::cerr << "Relayboard: checksum error";
				errorFlag = CHECKSUM_ERROR;
				return errorFlag;
			}
		}
	}

	return errorFlag;
}

//-----------------------------------------------
bool SerRelayBoard::init()
{
	int iRet;
	
	m_SerIO.setBaudRate(RS422_BAUDRATE);
	m_SerIO.setDeviceName( m_sNumComPort.c_str() );
	m_SerIO.setBufferSize(RS422_RX_BUFFERSIZE, RS422_TX_BUFFERSIZE);
	m_SerIO.setTimeout(RS422_TIMEOUT);

	iRet = m_SerIO.open();

	m_bComInit = true;

	return true;
}

//-----------------------------------------------
bool SerRelayBoard::reset()
{
	m_SerIO.close();
	m_bComInit = false;

	init();

	return true;
}

//-----------------------------------------------
bool SerRelayBoard::shutdown()
{
	m_SerIO.close();

	m_bComInit = false;
	
	return true;
}


//-----------------------------------------------
bool SerRelayBoard::isEMStop()
{
	if( (m_iRelBoardStatus & 0x0001) != 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//-----------------------------------------------
bool SerRelayBoard::isScannerStop()
{
	if( (m_iRelBoardStatus & 0x0002) != 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//-----------------------------------------------
int SerRelayBoard::sendRequest() {
	int errorFlag = NO_ERROR;
	int iNrBytesWritten;

	unsigned char cMsg[NUM_BYTE_SEND];
	
	m_Mutex.lock();
	
		convDataToSendMsg(cMsg);

		m_SerIO.purgeTx();

		iNrBytesWritten = m_SerIO.write((char*)cMsg, NUM_BYTE_SEND);
	
		if(iNrBytesWritten < NUM_BYTE_SEND) {
			//std::cerr << "Error in sending message to Relayboard over SerialIO, lost bytes during writing" << std::endl;
			errorFlag = GENERAL_SENDING_ERROR;
		}

	m_Mutex.unlock();

	return errorFlag;
}

int SerRelayBoard::setDigOut(int iChannel, bool bOn)
{
	switch( iChannel)
	{
	case 0:

		if(bOn) { m_iCmdRelayBoard |= CMD_SET_CHARGE_RELAY; }
		else { m_iCmdRelayBoard &= ~CMD_SET_CHARGE_RELAY; }
		
		break;

	case 1:

		if(bOn) { m_iCmdRelayBoard |= CMD_SET_RELAY1; }
		else { m_iCmdRelayBoard &= ~CMD_SET_RELAY1; }

		break;

	case 2:

		if(bOn) { m_iCmdRelayBoard |= CMD_SET_RELAY2; }
		else { m_iCmdRelayBoard &= ~CMD_SET_RELAY2; }

		break;

	case 3:

		if(bOn) { m_iCmdRelayBoard |= CMD_SET_RELAY3; }
		else { m_iCmdRelayBoard &= ~CMD_SET_RELAY3; }

		break;

	case 4:

		if(bOn) { m_iCmdRelayBoard |= CMD_SET_RELAY4; }
		else { m_iCmdRelayBoard &= ~CMD_SET_RELAY4; }

		break;

	case 5:

		if(bOn) { m_iCmdRelayBoard |= CMD_SET_RELAY5; }
		else { m_iCmdRelayBoard &= ~CMD_SET_RELAY5; }

		break;

	case 6:

		if(bOn) { m_iCmdRelayBoard |= CMD_SET_RELAY6; }
		else { m_iCmdRelayBoard &= ~CMD_SET_RELAY6; }

		break;

	default:

		return -1;
	}
	
	return 0;
}
//-----------------------------------------------
int SerRelayBoard::getAnalogIn(int* piAnalogIn)
{
	piAnalogIn[0] = m_iChargeCurrent;
	piAnalogIn[1] = m_iRelBoardBattVoltage;
	piAnalogIn[2] = m_iRelBoardTempSensor;
	piAnalogIn[3] = m_iRelBoardKeyPad;
	piAnalogIn[4] = m_iRelBoardAnalogIn[0];
	piAnalogIn[5] = m_iRelBoardAnalogIn[1];
	piAnalogIn[6] = m_iRelBoardAnalogIn[2];
	piAnalogIn[7] = m_iRelBoardAnalogIn[3];

	return 0;
}

//-----------------------------------------------
int SerRelayBoard::getDigIn()
{
	return m_iDigIn;
}

//-----------------------------------------------
void SerRelayBoard::convDataToSendMsg(unsigned char cMsg[])
{
	int i;
	int iCnt = 0;
	int iChkSum = 0;

	cMsg[iCnt++] = 1;//CMD_RELAISBOARD_GET_DATA;

	cMsg[iCnt++] = m_iConfigRelayBoard >> 8;
	cMsg[iCnt++] = m_iConfigRelayBoard;

	cMsg[iCnt++] = m_iCmdRelayBoard >> 8;
	cMsg[iCnt++] = m_iCmdRelayBoard;
	
	// fill remaining msg with 0's
	do
	{
		cMsg[iCnt++] = 0;
	}
	while(iCnt < (NUM_BYTE_SEND - 2));

	// calc checksum: summation of all bytes in the message
	for(i = 0; i < (NUM_BYTE_SEND - 2); i++)
	{
		iChkSum += cMsg[i];
	}

	cMsg[NUM_BYTE_SEND - 2] = iChkSum >> 8;
	cMsg[NUM_BYTE_SEND - 1] = iChkSum;

	// reset flags
	m_iCmdRelayBoard &= ~CMD_RESET_POS_CNT;
}

//-----------------------------------------------
bool SerRelayBoard::convRecMsgToData(unsigned char cMsg[])
{
	const int c_iStartCheckSum = NUM_BYTE_REC;
	
	int i;
	unsigned int iTxCheckSum;
	unsigned int iCheckSum;

	m_Mutex.lock();

	// test checksum: checksum should be sum of all bytes
	iTxCheckSum = (cMsg[c_iStartCheckSum + 1] << 8) | cMsg[c_iStartCheckSum];
	
	iCheckSum = 0;
	for(i = 0; i < c_iStartCheckSum; i++)
	{
		iCheckSum += cMsg[i];
	}

	if(iCheckSum != iTxCheckSum)
	{
		return false;
	}

	// convert data
	int iCnt = 0;

	//RelayboardStatus bytes contain EM-Stop and Scanner-Stop bits
	m_iRelBoardStatus = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	//unused at the moment
	m_iChargeCurrent = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	//unused at the moment
	m_iRelBoardBattVoltage = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	//unused at the moment	
	m_iRelBoardKeyPad = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	//unused at the moment
	for(i = 0; i < 4; i++)
	{
		m_iRelBoardAnalogIn[i] = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}

	//unused at the moment
	m_iRelBoardTempSensor = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;
	
	//Digital Inputs
	//unused at the moment
	m_iDigIn = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	//Throw away rest of the message, it was used for earlier purposes

	m_Mutex.unlock();
	return true;
}
