/* -----------------------------------------------------------------------------------
 *
 *		Copyright (c) 2007 Neobotix (www.neobotix.de)
 *
 *      This software is allowed to be used and modified only in association with a Neobotix
 *      robot platform. It is allowed to include the software into applications and
 *      to distribute it with a Neobotix robot platform. 
 *      It is not allowed to distribute the software without a Neobotix robot platform. 
 *
 *      This software is provided WITHOUT ANY WARRANTY; without even the
 *		implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *		PURPOSE. See the Neobotix License (Version 1.0) for more details.
 *
 *      You should have received a copy of the Neobotix License
 *      along with this software; if not, write to the 
 *      Gesellschaft fuer Produktionssysteme, Neobotix, Nobelstrasse 12, 70569 Stuttgart, Germany
 *
 * -----------------------------------------------------------------------------------
 */

#ifndef SerRelayBoard_INCLUDEDEF_H
#define SerRelayBoard_INCLUDEDEF_H

//-----------------------------------------------
#include <cob_relayboard/SerialIO.h>
#include <canopen_motor/DriveParam.h>
#include <cob_relayboard/Mutex.h>

//-----------------------------------------------

/**
 * Driver class for communication with a Neobotix RelayBoard.
 * Uses RS422 with 420 kBaud.
 */
class SerRelayBoard
{
public:
	
	SerRelayBoard(std::string ComPort);

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


	enum RelBoardReturns
	{
		NO_ERROR = 0,
		NOT_INITIALIZED = 1,
		GENERAL_SENDING_ERROR = 2,
		TOO_LESS_BYTES_IN_QUEUE = 3,
		NO_MESSAGES = 4, //for a long time, no message have been received, check com port!
		CHECKSUM_ERROR = 5,
	};

protected:
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

	std::string m_sNumComPort;

	void txCharArray();
	void rxCharArray();

	void convDataToSendMsg(unsigned char cMsg[]);
	bool convRecMsgToData(unsigned char cMsg[]);

	Mutex m_Mutex;
	
	//-----------------------
	// send data
	int m_iConfigRelayBoard;
	int m_iCmdRelayBoard;

	//-----------------------
	// rec data
	int m_iRelBoardStatus;
	int m_iChargeCurrent;
	int m_iRelBoardBattVoltage;
	int m_iRelBoardKeyPad;
	int m_iRelBoardAnalogIn[4];
	int m_iRelBoardTempSensor;
		int m_iDigIn;
	SerialIO m_SerIO;

	bool m_bComInit;
};


//-----------------------------------------------
#endif
