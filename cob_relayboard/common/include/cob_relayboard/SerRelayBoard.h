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
 

#ifndef SerRelayBoard_INCLUDEDEF_H
#define SerRelayBoard_INCLUDEDEF_H

//-----------------------------------------------
#include <cob_relayboard/SerialIO.h>
#include <cob_relayboard/Mutex.h>
#include <cob_relayboard/CmdRelaisBoard.h>

//-----------------------------------------------

/**
 * Driver class for communication with a Neobotix RelayBoard.
 * Uses RS422 with 420 kBaud.
 */
class SerRelayBoard
{
public:

	SerRelayBoard(std::string ComPort, int ProtocolVersion = 1);

	~SerRelayBoard();

	// Main control functions
	bool init();
	bool reset();
	bool shutdown();

	int evalRxBuffer(); //needs to be calles to read new data from relayboard
	int sendRequest(); //sends collected data and requests response

	//Services by relayboard
	int setDigOut(int iChannel, bool bOn);
	int getAnalogIn(int* piAnalogIn);
	int getDigIn();
	bool isEMStop();
	bool isScannerStop();
	int getBatteryVoltage()
	{
		return m_iRelBoardBattVoltage;
	}
	int getChargeCurrent()
	{
		return m_iChargeCurrent;
	}


	enum RelBoardReturns
	{
		NO_ERROR = 0,
		NOT_INITIALIZED = 1,
		GENERAL_SENDING_ERROR = 2,
		TOO_LESS_BYTES_IN_QUEUE = 3,
		NO_MESSAGES = 4, //for a long time, no message have been received, check com port!
		CHECKSUM_ERROR = 5,
	};

	enum RelBoardCmd
	{
		CMD_SET_CHARGE_RELAY = 1,
		CMD_RESET_POS_CNT = 2,
		CMD_QUICK_STOP = 4,
		CMD_SET_RELAY1 = 8,
		CMD_SET_RELAY2 = 16,
		CMD_SET_RELAY3 = 32,
		CMD_SET_RELAY4 = 64,
		CMD_SET_RELAY5 = 128,
		CMD_SET_RELAY6 = 256,
		CMD_ZERO_GYRO = 512
	};

	enum RelBoardConfig
	{
		CONFIG_HAS_IOBOARD = 1,
		CONFIG_HAS_USBOARD = 2,
		CONFIG_HAS_GYROBOARD = 4,
		CONFIG_HAS_RADARBOARD1 = 8,
		CONFIG_HAS_RADARBOARD2 = 16,
		CONFIG_HAS_DRIVES = 32,
	};

	enum TypeLCD
	{
		LCD_20CHAR_TEXT,
		LCD_60CHAR_TEXT,
		RELAY_BOARD_1_4
	};

protected:


	std::string m_sNumComPort;

	void txCharArray();
	void rxCharArray();

	void convDataToSendMsg(unsigned char cMsg[]);
	bool convRecMsgToData(unsigned char cMsg[]);

	Mutex m_Mutex;

	int m_iNumBytesSend;
	int m_iTypeLCD;

	unsigned char m_cTextDisplay[60];

	//relayboard 1.4:
	int m_iVelCmdMotRearRightEncS;
	int m_iVelCmdMotRearLeftEncS;
	char m_cSoftEMStop;
	char m_cDebugRearRight[4];
	int m_iPosMeasMotRearRightEnc;
	int m_iVelMeasMotRearRightEncS;
	int m_iPosMeasMotRearLeftEnc;
	char m_cDebugRearLeft[4];
	int m_iVelMeasMotRearLeftEncS;
	int m_iMotRearRightStatus;
	int m_iMotRearLeftStatus;
	double m_dLastPosRearRight;
	double m_dLastPosRearLeft;

	//-----------------------
	// send data

	// RelayBoard
	int m_iConfigRelayBoard;
	int m_iCmdRelayBoard;

	// IOBoard
	int m_iIOBoardDigOut;

	// MotCtrlBoards
	int m_iVelCmdMotRightEncS;
	int m_iVelCmdMotLeftEncS;

	// USBoard
	int m_iUSBoardSensorActive;

	//-----------------------
	// rec data
	int m_iRelBoardStatus;
	int m_iChargeCurrent;
	int m_iRelBoardBattVoltage;
	int m_iRelBoardKeyPad;
	int m_iRelBoardAnalogIn[4];
	int m_iRelBoardTempSensor;

	int m_iDigIn;
	int m_iProtocolVersion;
	int m_NUM_BYTE_SEND;

	SerialIO m_SerIO;

	bool m_bComInit;
};


//-----------------------------------------------
#endif
