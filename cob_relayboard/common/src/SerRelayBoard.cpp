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


#include <math.h>
#include <cob_relayboard/SerRelayBoard.h>
#include <iostream>

//-----------------------------------------------


#define NUM_BYTE_SEND 79 //Total amount of data sent to relayboard in one message, is now passed and set as protocol-version argument in constructor

#define RS422_BAUDRATE 420000
#define RS422_RX_BUFFERSIZE 1024
#define RS422_TX_BUFFERSIZE 1024

#define RS422_TIMEOUT 0.025

#define NUM_BYTE_REC_MAX 120
#define NUM_BYTE_REC_HEADER 4 //Header bytes, which are used to identify the beginning of a new received message {0x02, 0x80, 0xD6, 0x02}
#define NUM_BYTE_REC_CHECKSUM 2 //checksum for message, that is built as the sum of all data bytes contained in the message
#define NUM_BYTE_REC 104 //Total amount of data bytes in a received message (from the relayboard)

#define NUM_BYTE_SEND_RELAYBOARD_14 88
#define NUM_BYTE_REC_RELAYBOARD_14 124


//-----------------------------------------------
SerRelayBoard::SerRelayBoard(std::string ComPort, int ProtocolVersion)
{
	m_iProtocolVersion = ProtocolVersion;
	if(m_iProtocolVersion == 1)
		m_NUM_BYTE_SEND = 50;
	else if(m_iProtocolVersion == 2)
	{	m_NUM_BYTE_SEND = 79;
		m_iTypeLCD = LCD_60CHAR_TEXT;
	}
	else if(m_iProtocolVersion == 3)
	{
		m_NUM_BYTE_SEND = NUM_BYTE_SEND_RELAYBOARD_14;
		m_iTypeLCD = RELAY_BOARD_1_4;
	}
	m_bComInit = false;
	m_sNumComPort = ComPort;

	m_iRelBoardBattVoltage = 0;
	m_iConfigRelayBoard = 0;
	m_iRelBoardKeyPad = 0xFFFF;
	m_iCmdRelayBoard = 0;
	m_iDigIn = 0;
	m_cSoftEMStop = 0;

}

//-----------------------------------------------
SerRelayBoard::~SerRelayBoard()
{
	m_SerIO.closeIO();
}

//-----------------------------------------------
int SerRelayBoard::evalRxBuffer()
{
	static int siNoMsgCnt = 0;

	int iNumByteRec = NUM_BYTE_REC;
	if(m_iTypeLCD == RELAY_BOARD_1_4)
	{
		iNumByteRec = NUM_BYTE_REC_RELAYBOARD_14;
	}

	const int c_iNrBytesMin = NUM_BYTE_REC_HEADER + iNumByteRec + NUM_BYTE_REC_CHECKSUM;
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
	m_SerIO.setBaudRate(RS422_BAUDRATE);
	m_SerIO.setDeviceName( m_sNumComPort.c_str() );
	m_SerIO.setBufferSize(RS422_RX_BUFFERSIZE, RS422_TX_BUFFERSIZE);
	m_SerIO.setTimeout(RS422_TIMEOUT);

	m_SerIO.openIO();

	m_bComInit = true;

	return true;
}

//-----------------------------------------------
bool SerRelayBoard::reset()
{
	m_SerIO.closeIO();
	m_bComInit = false;

	init();

	return true;
}

//-----------------------------------------------
bool SerRelayBoard::shutdown()
{
	m_SerIO.closeIO();

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

	unsigned char cMsg[m_NUM_BYTE_SEND];

	m_Mutex.lock();

		convDataToSendMsg(cMsg);

		m_SerIO.purgeTx();

		iNrBytesWritten = m_SerIO.writeIO((char*)cMsg, m_NUM_BYTE_SEND);

		if(iNrBytesWritten < m_NUM_BYTE_SEND) {
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

void SerRelayBoard::convDataToSendMsg(unsigned char cMsg[])
{
	int i;
	static int j = 0;
	int iCnt = 0;
	int iChkSum = 0;

	if (m_cSoftEMStop & 0x02)
	{
		if (j == 1)
		{
			m_cSoftEMStop &= 0xFD;
			j = 0;
		}
		else if (j == 0)
		{
			j = 1;
		}
	}

	cMsg[iCnt++] = CMD_RELAISBOARD_GET_DATA;

	cMsg[iCnt++] = m_iConfigRelayBoard >> 8;
	cMsg[iCnt++] = m_iConfigRelayBoard;

	cMsg[iCnt++] = m_iCmdRelayBoard >> 8;
	cMsg[iCnt++] = m_iCmdRelayBoard;

	cMsg[iCnt++] = m_iIOBoardDigOut >> 8;
	cMsg[iCnt++] = m_iIOBoardDigOut;

	cMsg[iCnt++] = m_iVelCmdMotRightEncS >> 24;
	cMsg[iCnt++] = m_iVelCmdMotRightEncS >> 16;
	cMsg[iCnt++] = m_iVelCmdMotRightEncS >> 8;
	cMsg[iCnt++] = m_iVelCmdMotRightEncS;

	cMsg[iCnt++] = m_iVelCmdMotLeftEncS >> 24;
	cMsg[iCnt++] = m_iVelCmdMotLeftEncS >> 16;
	cMsg[iCnt++] = m_iVelCmdMotLeftEncS >> 8;
	cMsg[iCnt++] = m_iVelCmdMotLeftEncS;

	if(m_iTypeLCD == RELAY_BOARD_1_4)
	{
		cMsg[iCnt++] = m_iVelCmdMotRearRightEncS >> 24;
		cMsg[iCnt++] = m_iVelCmdMotRearRightEncS >> 16;
		cMsg[iCnt++] = m_iVelCmdMotRearRightEncS >> 8;
		cMsg[iCnt++] = m_iVelCmdMotRearRightEncS;

		cMsg[iCnt++] = m_iVelCmdMotRearLeftEncS >> 24;
		cMsg[iCnt++] = m_iVelCmdMotRearLeftEncS >> 16;
		cMsg[iCnt++] = m_iVelCmdMotRearLeftEncS >> 8;
		cMsg[iCnt++] = m_iVelCmdMotRearLeftEncS;
	}

	cMsg[iCnt++] = m_iUSBoardSensorActive >> 8;
	cMsg[iCnt++] = m_iUSBoardSensorActive;

	if(m_iTypeLCD == LCD_20CHAR_TEXT)
	{
		for(i = 0; i < 20; i++)
		{
			cMsg[iCnt++] = m_cTextDisplay[i];
		}

		// fill remaining msg with 0's
		do
		{
			cMsg[iCnt++] = 0;
		}
		while(iCnt < (m_NUM_BYTE_SEND - 2));
	}
	else
	{
		for(i = 0; i < 60; i++)
		{
			cMsg[iCnt++] = m_cTextDisplay[i];
		}
	}

	if(m_iTypeLCD == RELAY_BOARD_1_4)
	{
		cMsg[iCnt++] = m_cSoftEMStop;
	}
	// calc checksum
	for(i = 0; i < (m_NUM_BYTE_SEND - 2); i++)
	{
		iChkSum %= 0xFF00;
		iChkSum += cMsg[i];
	}

	cMsg[m_NUM_BYTE_SEND - 2] = iChkSum >> 8;
	cMsg[m_NUM_BYTE_SEND - 1] = iChkSum;

	// reset flags
	m_iCmdRelayBoard &= ~CMD_RESET_POS_CNT;

}


//-----------------------------------------------
/*void SerRelayBoard::convDataToSendMsg(unsigned char cMsg[])
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
	while(iCnt < (m_NUM_BYTE_SEND - 2));

	// calc checksum: summation of all bytes in the message
	for(i = 0; i < (m_NUM_BYTE_SEND - 2); i++)
	{
		iChkSum += cMsg[i];
	}

	cMsg[m_NUM_BYTE_SEND - 2] = iChkSum >> 8;
	cMsg[m_NUM_BYTE_SEND - 1] = iChkSum;

	// reset flags
	m_iCmdRelayBoard &= ~CMD_RESET_POS_CNT;
}
*/
//-----------------------------------------------
bool SerRelayBoard::convRecMsgToData(unsigned char cMsg[])
{

	int iNumByteRec = NUM_BYTE_REC;
	if(m_iTypeLCD == LCD_20CHAR_TEXT)
	{
		iNumByteRec = NUM_BYTE_REC;
	}
	if(m_iTypeLCD == LCD_60CHAR_TEXT)
	{
		iNumByteRec = NUM_BYTE_REC;
	}
	if(m_iTypeLCD == RELAY_BOARD_1_4)
	{
		iNumByteRec = NUM_BYTE_REC_RELAYBOARD_14;
	}

	const int c_iStartCheckSum = iNumByteRec;

	int i;
	unsigned int iTxCheckSum;
	unsigned int iCheckSum;

	m_Mutex.lock();

	// test checksum: checksum should be sum of all bytes
	iTxCheckSum = (cMsg[c_iStartCheckSum + 1] << 8) | cMsg[c_iStartCheckSum];

	iCheckSum = 0;
	for(i = 0; i < c_iStartCheckSum; i++)
	{
		iCheckSum %= 0xFF00;
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
