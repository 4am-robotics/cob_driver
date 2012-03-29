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



#include <math.h>

#include "neo_SerRelayBoard/SerRelayBoard.h"


//-----------------------------------------------


#define NUM_BYTE_SEND_20CHAR_LCD 50
#define NUM_BYTE_SEND_60CHAR_LCD 79

#define RS422_BAUDRATE 420000
#define RS422_RX_BUFFERSIZE 1024
#define RS422_TX_BUFFERSIZE 1024

#define RS422_TIMEOUT 0.025

#define NUM_BYTE_REC_MAX 120
#define NUM_BYTE_REC_HEADER 4
#define NUM_BYTE_REC_CHECKSUM 2
#define NUM_BYTE_REC 104

#define NEO_PI 3.14159265358979323846
#define DEG2RAD(x) NEO_PI/180 * x


#define OUTPUTERROR(...)

SerRelayBoard::SerRelayBoard()
{
	autoSendRequest = false;	//requests are being send when calling sendRequest();
}

/*SerRelayBoard::SerRelayBoard(int iTypeLCD, std::string pathToConf)
{
	autoSendRequest = true;		//requests are being send when calling setWheelVel(...);
	setStartValues(iTypeLCD); //TODO: backwards compatibility
}
*/

void SerRelayBoard::readConfig(	int iTypeLCD,std::string pathToConf, std::string sNumComPort, 	int hasMotorRight, 
				int hasMotorLeft, int hasIOBoard, int hasUSBoard, int hasRadarBoard, int hasGyroBoard, 
				double quickfix1, double quickfix2, DriveParam driveParamLeft, DriveParam driveParamRight
			)
{
	int i, iHasBoard, iHasMotorRight, iHasMotorLeft;
	m_bComInit = false;


	//RelayBoard
	iHasBoard = 0;
	m_iRelBoardBattVoltage = 0;
	m_iConfigRelayBoard = 0;
	m_iRelBoardKeyPad = 0xFFFF;

	m_iTypeLCD = iTypeLCD;
	if(m_iTypeLCD == LCD_20CHAR_TEXT)
	{
		m_iNumBytesSend = NUM_BYTE_SEND_20CHAR_LCD;
	}
	else
	{
		m_iNumBytesSend = NUM_BYTE_SEND_60CHAR_LCD;
	}

	m_sNumComPort = sNumComPort;

	iHasMotorRight = hasMotorRight;
	iHasMotorLeft = hasMotorLeft;
	if( (iHasMotorRight != 0) || (iHasMotorLeft != 0) )
	{
		m_iConfigRelayBoard |= CONFIG_HAS_DRIVES;
	}
	
	iHasBoard = hasIOBoard;
	if(iHasBoard == 1)
	{
		m_iConfigRelayBoard |= CONFIG_HAS_IOBOARD;
	}

	iHasBoard = hasUSBoard;
	if(iHasBoard == 1)
	{
		m_iConfigRelayBoard |= CONFIG_HAS_USBOARD;
	}
	
	iHasBoard = hasRadarBoard;
	if(iHasBoard == 1)
	{
		m_iConfigRelayBoard |= CONFIG_HAS_RADARBOARD1;
		m_iConfigRelayBoard |= CONFIG_HAS_RADARBOARD2;
	}

	iHasBoard = hasGyroBoard;
	if(iHasBoard == 1)
	{
		m_iConfigRelayBoard |= CONFIG_HAS_GYROBOARD;
	}

	m_iCmdRelayBoard = 0;
	m_iRelBoardStatus = 0;

	// IOBoard
	m_iIOBoardBattVoltage = 0;
	m_iIOBoardDigIn = 0;
	m_iIOBoardDigOut = 0;
	for(i = 0; i < 8; i++) { m_iIOBoardAnalogIn[i] = 0; }

	quickFix[0] = quickfix1;
	quickFix[1] = quickfix2;

	// drive parameters
	
	m_DriveParamLeft = driveParamLeft;
	m_DriveParamRight = driveParamRight;

	//
	m_iVelCmdMotRightEncS = 0;
	m_iVelCmdMotLeftEncS = 0;
	m_iPosMeasMotRightEnc = 0;
	m_iVelMeasMotRightEncS = 0;
	m_iPosMeasMotLeftEnc = 0;
	m_iVelMeasMotLeftEncS = 0;

	// GyroBoard
	m_bGyroBoardZeroGyro = false;
	m_iGyroBoardAng = 0;
	for(i = 0; i < 3; i++) { m_iGyroBoardAcc[i]; }
	
	// RadarBoard
	for(i = 0; i < 4; i++) { m_iRadarBoardVel[i] = 0; }
	
	// USBoard
	m_iUSBoardSensorActive = 0xFFFF;

	for(i = 0; i < 16; i++) { m_iUSBoardSensorData[i] = 0; }
	for(i = 0; i < 4; i++) { m_iUSBoardAnalogData[i] = 0; }

	m_dLastPosRight = 0;
	m_dLastPosLeft = 0;

	for(i = 0; i < 60; i++) { m_cTextDisplay[i] = 0; }
}





//-----------------------------------------------
SerRelayBoard::~SerRelayBoard()
{
	m_SerIO.close();
}

//-----------------------------------------------
void SerRelayBoard::readConfiguration()
{

}

//-----------------------------------------------
int SerRelayBoard::evalRxBuffer()
{
	int errorFlag = NO_ERROR;
	static int siNoMsgCnt = 0;

	double dDltT;
	
	const int c_iNrBytesMin = NUM_BYTE_REC_HEADER + NUM_BYTE_REC + NUM_BYTE_REC_CHECKSUM;
	const int c_iSizeBuffer = 4096;

	int i;
	int iNrBytesInQueue, iNrBytesRead, iDataStart;
	unsigned char cDat[c_iSizeBuffer];
	unsigned char cTest[4] = {0x02, 0x80, 0xD6, 0x02};

	//m_CurrTime = ros::Time::now();
	//dDltT = m_CurrTime.toSec()- m_LastTime.toSec();

	if( !m_bComInit ) return 0;

	//enough data in queue?
	iNrBytesInQueue = m_SerIO.getSizeRXQueue();

	if(iNrBytesInQueue < c_iNrBytesMin)
	{
		siNoMsgCnt++;
		if(siNoMsgCnt > 29)
		{
			siNoMsgCnt = 0;
			errorFlag = NO_MESSAGES;
		}  else errorFlag = TOO_LESS_BYTES_IN_QUEUE;


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
				errorFlag = CHECKSUM_ERROR;
				return errorFlag;
			}
		}
	}

	return errorFlag;
}

//-----------------------------------------------
bool SerRelayBoard::init() { //CareOBots Syntax Wrapper
	return initPltf();
};
bool SerRelayBoard::initPltf()
{
	int iRet;
	m_SerIO.setBaudRate(RS422_BAUDRATE);
	m_SerIO.setDeviceName( m_sNumComPort.c_str() );
	m_SerIO.setBufferSize(RS422_RX_BUFFERSIZE, RS422_TX_BUFFERSIZE);
	m_SerIO.setTimeout(RS422_TIMEOUT);
	iRet = m_SerIO.open();
	m_bComInit = true;

	m_iCmdRelayBoard |= CMD_RESET_POS_CNT;

	return true;
}

//-----------------------------------------------
bool SerRelayBoard::reset(){
	return resetPltf();
}

bool SerRelayBoard::resetPltf()
{
	m_SerIO.close();
	m_bComInit = false;

	init();
	return true;
}

//-----------------------------------------------
bool SerRelayBoard::shutdown()
{
	return shutdownPltf();
}


bool SerRelayBoard::shutdownPltf()
{
	m_SerIO.close();
	m_bComInit = false;
	return true;
}

//-----------------------------------------------
bool SerRelayBoard::isComError()
{
	return false;	
}

//-----------------------------------------------
bool SerRelayBoard::isDriveError()
{
	return false;	
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
void SerRelayBoard::sendNetStartCanOpen()
{

}

//new Syntax:
int SerRelayBoard::sendRequest()
{
	if(!autoSendRequest){
		int errorFlag = NO_ERROR;
		int iNrBytesWritten;

		unsigned char cMsg[m_iNumBytesSend];
	
		m_Mutex.lock();
	
			convDataToSendMsg(cMsg);

			m_SerIO.purgeTx();

			iNrBytesWritten = m_SerIO.write((char*)cMsg,m_iNumBytesSend);
	
			if(iNrBytesWritten < m_iNumBytesSend) {
				errorFlag = GENERAL_SENDING_ERROR;
			}

			//m_LastTime = ros::Time::now();
		m_Mutex.unlock();
		return errorFlag;
	} else {
		OUTPUTERROR("You are running the depreced mode for backward compability, that's why sendRequest is being handled by setWheelVel()");
	}
};


//-----------------------------------------------
// MotCtrlBoard
//-----------------------------------------------

//-----------------------------------------------
int SerRelayBoard::disableBrake(int iCanIdent, bool bDisabled)
{
	return 0;
}

//-----------------------------------------------
int SerRelayBoard::setWheelVel(int iCanIdent, double dVelWheel, bool bQuickStop)
{		
	static bool bDataMotRight = false;
	static bool bDataMotLeft = false;

	bool bVelLimited = false;
	int iNrBytesWritten;

	unsigned char cMsg[NUM_BYTE_SEND_60CHAR_LCD];
	
	m_Mutex.lock();
	
	if(iCanIdent == CANNODE_MOTORRIGHT)
	{
		m_iVelCmdMotRightEncS = m_DriveParamRight.getSign()	*
			m_DriveParamRight.convRadSToIncrPerPeriod(dVelWheel);

/* TODO:
		if(MathSup::limit(&m_iVelCmdMotRightEncS, (int)m_DriveParamRight.getVelMax()) != 0)
		{
			LOGALERT("vel right limited");
		}
*/
		bDataMotRight = true;
	}

	if(iCanIdent == CANNODE_MOTORLEFT)
	{
		m_iVelCmdMotLeftEncS = m_DriveParamLeft.getSign()	*
			m_DriveParamLeft.convRadSToIncrPerPeriod(dVelWheel);


/* TODO:
		if(MathSup::limit(&m_iVelCmdMotLeftEncS, (int)m_DriveParamLeft.getVelMax()) != 0)
		{
			LOGALERT("vel left limited");
		}
*/
	
		bDataMotLeft = true;
	}

	if(autoSendRequest && bDataMotRight && bDataMotLeft)
	{
		bDataMotRight = bDataMotLeft = false;

		if(bQuickStop)
		{
			m_iCmdRelayBoard |= CMD_QUICK_STOP;
		}
		else
		{
			m_iCmdRelayBoard &= ~CMD_QUICK_STOP;
		}
		
		convDataToSendMsg(cMsg);

		m_SerIO.purgeTx();

		// send data
		iNrBytesWritten = m_SerIO.write((char*)cMsg, m_iNumBytesSend);
		if(iNrBytesWritten < m_iNumBytesSend)
		{
			//std::cerr << "only " << iNrBytesWritten << " bytes written" << std::endl;
		}

		//m_LastTime = ros::Time::now();
	}

	m_Mutex.unlock();

	return iNrBytesWritten;
}
//-----------------------------------------------
int SerRelayBoard::setWheelPosVel(int iCanIdent, double dPos, double dVel, bool bQuickStop)
{
	return 0;
}

//-----------------------------------------------
int SerRelayBoard::requestMotPosVel(int iCanIdent)
{
	return 0;
}

//-----------------------------------------------
void SerRelayBoard::requestDriveStatus()
{

}

//-----------------------------------------------
int SerRelayBoard::getWheelPosVel(	int iCanIdent, double* pdAngWheel, double* pdVelWheel)
{
	m_Mutex.lock();
	
	if(iCanIdent == CANNODE_MOTORRIGHT)
	{
		*pdAngWheel = m_DriveParamRight.getSign() * m_DriveParamRight.convIncrToRad(m_iPosMeasMotRightEnc) * quickFix[0];	
		*pdVelWheel = m_DriveParamRight.getSign() * m_DriveParamRight.convIncrPerPeriodToRadS(m_iVelMeasMotRightEncS) * quickFix[0];
	}

	if(iCanIdent == CANNODE_MOTORLEFT)
	{
		*pdAngWheel = m_DriveParamLeft.getSign() * m_DriveParamLeft.convIncrToRad(m_iPosMeasMotLeftEnc) * quickFix[1];	
		*pdVelWheel = m_DriveParamLeft.getSign() * m_DriveParamLeft.convIncrPerPeriodToRadS(m_iVelMeasMotLeftEncS) * quickFix[1];
	}

	m_Mutex.unlock();

	return 0;
}

//-----------------------------------------------
int SerRelayBoard::getWheelDltPosVel(	int iCanIdent, double* pdDltAng, double* pdVelWheel)
{
	double dCurrPos;

	m_Mutex.lock();
	
	if(iCanIdent == CANNODE_MOTORRIGHT)
	{
		dCurrPos = m_DriveParamRight.getSign() * m_DriveParamRight.convIncrToRad(m_iPosMeasMotRightEnc) * quickFix[0];
		
		*pdDltAng = dCurrPos - m_dLastPosRight;
		m_dLastPosRight = dCurrPos;

		*pdVelWheel = m_DriveParamRight.getSign() * m_DriveParamRight.convIncrPerPeriodToRadS(m_iVelMeasMotRightEncS) * quickFix[0];
	}

	if(iCanIdent == CANNODE_MOTORLEFT)
	{
		dCurrPos = m_DriveParamLeft.getSign() * m_DriveParamLeft.convIncrToRad(m_iPosMeasMotLeftEnc) * quickFix[1];

		*pdDltAng = dCurrPos - m_dLastPosLeft;
		m_dLastPosLeft = dCurrPos;

		*pdVelWheel = m_DriveParamLeft.getSign() * m_DriveParamLeft.convIncrPerPeriodToRadS(m_iVelMeasMotLeftEncS) * quickFix[1];
	}

	m_Mutex.unlock();

	return 0;
}

//-----------------------------------------------
void SerRelayBoard::getStatus(int iCanIdent, int* piStatus, int* piTempCel)
{
	if(iCanIdent == CANNODE_MOTORLEFT)
	{
		*piStatus = m_iMotLeftStatus;
	}
	else
	{
		*piStatus = m_iMotRightStatus;
	}

	*piTempCel = 0;
}

//-----------------------------------------------
int SerRelayBoard::execHoming(int CanIdent)
{
	// NOT IMPLEMENTED !!!!
	return 0;
}

//-----------------------------------------------
// RelayBoard
//-----------------------------------------------


int SerRelayBoard::setRelayBoardDigOut(int iChannel, bool bOn)
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
int SerRelayBoard::getRelayBoardAnalogIn(int* piAnalogIn)
{
	piAnalogIn[0] = m_iChargeCurrent;
	piAnalogIn[1] = m_iRelBoardBattVoltage;
	piAnalogIn[2] = m_iRelBoardTempSensor;
	piAnalogIn[3] = m_iRelBoardKeyPad;
	piAnalogIn[4] = m_iRelBoardIRSensor[0];
	piAnalogIn[5] = m_iRelBoardIRSensor[1];
	piAnalogIn[6] = m_iRelBoardIRSensor[2];
	piAnalogIn[7] = m_iRelBoardIRSensor[3];

	return 0;
}

//-----------------------------------------------
int SerRelayBoard::getRelayBoardDigIn()
{
	//is transmitted in keypaddata
	//m_iRelBoardKeyPad & 
	if((~m_iRelBoardKeyPad) & 0x20)
		//first digin 31
		return 1;
	if((~m_iRelBoardKeyPad) & 0x10)
		//second digin 47
		return 2;
	else
		return 0;

}

//-----------------------------------------------
// IOBoard
//-----------------------------------------------

//-----------------------------------------------
void SerRelayBoard::requestIOBoardData()
{
}

//-----------------------------------------------
void SerRelayBoard::requestIOBoardAnalogIn()
{
}

//-----------------------------------------------
void SerRelayBoard::getIOBoardJoyValWheelMean(double* pdVelWheelLeftMean, double* pdVelWheelRightMean)
{
	*pdVelWheelLeftMean = 0;
	*pdVelWheelRightMean = 0;
}

//-----------------------------------------------
void SerRelayBoard::getIOBoardJoyValNorm(double* pdJoyXNorm, double* pdJoyYNorm)
{
	*pdJoyXNorm = 0;
	*pdJoyYNorm = 0;
}

//-----------------------------------------------
int SerRelayBoard::getIOBoardBattVoltage()
{
	return m_iIOBoardBattVoltage;
}

//-----------------------------------------------
int SerRelayBoard::getIOBoardDigIn()
{
	return m_iIOBoardDigIn;
}

//-----------------------------------------------
int SerRelayBoard::getIOBoardDigOut()
{
	return m_iIOBoardDigOut;
}

//-----------------------------------------------
int SerRelayBoard::setIOBoardDigOut(int iChannel, bool bVal)
{
	int iMask;

	iMask = (1 << iChannel);
	
	if(bVal)
	{
		m_iIOBoardDigOut |= iMask;
	}
	else
	{
		m_iIOBoardDigOut &= ~iMask;
	}
	
	return 0;	
}

//-----------------------------------------------
int SerRelayBoard::getIOBoardAnalogIn(int* piAnalogIn)
{
	int i;

	m_Mutex.lock();
	
	for(i = 0; i < 8; i++)
	{
		piAnalogIn[i] = m_iIOBoardAnalogIn[i];
	}

	m_Mutex.unlock();

	return 0;
}

//-----------------------------------------------
void SerRelayBoard::writeIOBoardLCD(int iLine, int iColumn, const std::string& sText)
{
	int i, iSize;

	iSize = sText.size();

	if(m_iTypeLCD == LCD_20CHAR_TEXT)
	{
		for(i = 0; i < 20; i++)
		{
			if(i < iSize)
			{
				m_cTextDisplay[i] = sText[i];
			}
			else
			{
				m_cTextDisplay[i] = 0;
			}
		}
	}
	else
	{
		for(i = 0; i < 60; i++)
		{
			if(i < iSize)
			{
				m_cTextDisplay[i] = sText[i];
			}
			else
			{
				m_cTextDisplay[i] = ' ';
			}
		}
	}
}

//-----------------------------------------------
//USBoard
//-----------------------------------------------

//-----------------------------------------------
int SerRelayBoard::startUS(int iChannelActive)
{
	m_iUSBoardSensorActive = iChannelActive;
	
	return 0;
}
//-----------------------------------------------
int SerRelayBoard::stopUS()
{
	m_iUSBoardSensorActive = 0x00;

	return 0;
}
//-----------------------------------------------
void SerRelayBoard::requestUSBoardData1To8()
{
}

//-----------------------------------------------
int SerRelayBoard::getUSBoardData1To8(int* piUSDistMM)
{
	int i;

	m_Mutex.lock();
	
	for(i = 0; i < 8; i++)
	{
		piUSDistMM[i] = 10 * m_iUSBoardSensorData[i];
	}

	m_Mutex.unlock();

	return 0;
}

//-----------------------------------------------
void SerRelayBoard::requestUSBoardData9To16()
{
}

//-----------------------------------------------
int SerRelayBoard::getUSBoardData9To16(int* piUSDistMM)
{
	int i;

	m_Mutex.lock();

	for(i = 0; i < 8; i++)
	{
		piUSDistMM[i] = 10 * m_iUSBoardSensorData[i + 8];
	}

	m_Mutex.unlock();
	
	return 0;
}

//-----------------------------------------------
void SerRelayBoard::requestUSBoardAnalogIn()
{
}

//-----------------------------------------------
void SerRelayBoard::getUSBoardAnalogIn(int* piAnalogIn)
{
	int i;

	m_Mutex.lock();

	for(i = 0; i < 4; i++)
	{
		piAnalogIn[i] = m_iUSBoardAnalogData[i];
	}

	m_Mutex.unlock();
}




//-----------------------------------------------
// GyroBoard
//-----------------------------------------------

//-----------------------------------------------
void SerRelayBoard::zeroGyro(bool bZeroActive)
{
	if(bZeroActive)
	{
		m_iCmdRelayBoard |= CMD_ZERO_GYRO;
	}
	else
	{
		m_iCmdRelayBoard &= ~CMD_ZERO_GYRO;
	}
}

//-----------------------------------------------
int SerRelayBoard::getGyroBoardAng(double* pdAngRad, double dAcc[])
{
	const double cdGyroScale = 1 / 520.5;

	int i;
	double dAng;
	
	m_Mutex.lock();
	
	dAng = m_iGyroBoardAng * cdGyroScale; // gyroboard transmits gyro value in degrees
	*pdAngRad = DEG2RAD(dAng);

	for(i = 0; i < 3; i++)
	{
		dAcc[i] = m_iGyroBoardAcc[i];
	}

	m_Mutex.unlock();
	
	return 0;
}
int SerRelayBoard::getGyroBoardAngBoost(double* pdAngRad, boost::array<double, 3u>& dAcc)
{
	const double cdGyroScale = 1 / 520.5;

	int i;
	double dAng;
	
	m_Mutex.lock();
	
	dAng = m_iGyroBoardAng * cdGyroScale; // gyroboard transmits gyro value in degrees
	*pdAngRad = DEG2RAD(dAng);

	for(i = 0; i < 3; i++)
	{
		dAcc[i] = m_iGyroBoardAcc[i];
	}

	m_Mutex.unlock();
	
	return 0;
}


//-----------------------------------------------
int SerRelayBoard::getGyroBoardDltAng(double* pdAng, double dAcc[])
{
	return 0;
}

//-----------------------------------------------
// RadarBoard
//-----------------------------------------------
void SerRelayBoard::requestRadarBoardData()
{
}

//-----------------------------------------------
int SerRelayBoard::getRadarBoardData(double* pdVelMMS)
{
	int i;

	m_Mutex.lock();
	
	for(i = 0; i < 4; i++)
	{
		pdVelMMS[i] = m_iRadarBoardVel[i];

	}

	m_Mutex.unlock();
	
	return 0;
}

//-----------------------------------------------
// CanClientGeneric
//-----------------------------------------------
void SerRelayBoard::addGenericCANListeningId(int id)
{
}

//-----------------------------------------------
void SerRelayBoard::removeGenericCANListeningId(int id)
{
}

//-----------------------------------------------
void SerRelayBoard::getGenericCANMessages(std::vector<CANTimedMessage>& pMessages)
{
}

//-----------------------------------------------
void SerRelayBoard::sendGenericCANMessage(CanMsg& message)
{
}


//-----------------------------------------------
void SerRelayBoard::convDataToSendMsg(unsigned char cMsg[])
{
	int i;
	int iCnt = 0;
	int iChkSum = 0;

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
		while(iCnt < (m_iNumBytesSend - 2));
	}
	else
	{
		for(i = 0; i < 60; i++)
		{
			cMsg[iCnt++] = m_cTextDisplay[i];
		}
	}

	// calc checksum
	for(i = 0; i < (m_iNumBytesSend - 2); i++)
	{
		iChkSum += cMsg[i];
	}

	cMsg[m_iNumBytesSend - 2] = iChkSum >> 8;
	cMsg[m_iNumBytesSend - 1] = iChkSum;

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

	// test checksum
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

	// RelayBoard
	
	m_iRelBoardStatus = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;
	
	m_iChargeCurrent = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];

	iCnt += 2;

	m_iRelBoardBattVoltage = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;
	
	m_iRelBoardKeyPad = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	for(i = 0; i < 4; i++)
	{
		m_iRelBoardIRSensor[i] = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}

	m_iRelBoardTempSensor = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;
	
	// IOBoard
	
	m_iIOBoardDigIn = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	for(i = 0; i < 8; i++)
	{
		m_iIOBoardAnalogIn[i] = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}

	m_iIOBoardStatus =  (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	// Motion control boards
	
	m_iPosMeasMotRightEnc =	cMsg[iCnt] |
							(cMsg[iCnt + 1] << 8) |
							(cMsg[iCnt + 2] << 16) |
							(cMsg[iCnt + 3] << 24);

	m_cDebugRight[0] = cMsg[iCnt];
	m_cDebugRight[1] = cMsg[iCnt + 1];
	m_cDebugRight[2] = cMsg[iCnt + 2];
	m_cDebugRight[3] = cMsg[iCnt + 3];

	iCnt += 4;

	m_iVelMeasMotRightEncS =cMsg[iCnt] |
							(cMsg[iCnt + 1] << 8) |
							(cMsg[iCnt + 2] << 16) |
							(cMsg[iCnt + 3] << 24);
	iCnt += 4;
	
	m_iPosMeasMotLeftEnc =	cMsg[iCnt] |
							(cMsg[iCnt + 1] << 8) |
							(cMsg[iCnt + 2] << 16) |
							(cMsg[iCnt + 3] << 24);

	m_cDebugLeft[0] = cMsg[iCnt];
	m_cDebugLeft[1] = cMsg[iCnt + 1];
	m_cDebugLeft[2] = cMsg[iCnt + 2];
	m_cDebugLeft[3] = cMsg[iCnt + 3];

	iCnt += 4;

	m_iVelMeasMotLeftEncS = cMsg[iCnt] |
							(cMsg[iCnt + 1] << 8) |
							(cMsg[iCnt + 2] << 16) |
							(cMsg[iCnt + 3] << 24);
	iCnt += 4;

	m_iMotRightStatus = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	m_iMotLeftStatus = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	// GyroBoard
	
	m_iGyroBoardAng =	(cMsg[iCnt] << 24) |
						(cMsg[iCnt + 1] << 16) |
						(cMsg[iCnt + 2] << 8) |
						cMsg[iCnt + 3];
	iCnt += 4;

	for(i = 0; i < 3; i++)
	{
		m_iGyroBoardAcc[i] = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}

	m_iGyroBoardStatus =  (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	// RadarBoard
	
	for(i = 0; i < 3; i++)
	{
		m_iRadarBoardVel[i] = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}

	m_iRadarBoardStatus = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;
	
	// USBoard
	
	for(i = 0; i < 16; i++)
	{
		m_iUSBoardSensorData[i] = (cMsg[iCnt++]);
	}
	
	for(i = 0; i < 4; i++)
	{
		m_iUSBoardAnalogData[i] = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}

	m_iUSBoardStatus = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;
	
	m_Mutex.unlock();

	if( iCnt >= NUM_BYTE_REC_MAX )
	{
		OUTPUTERROR("msg size too small");
	}

	return true;
}
