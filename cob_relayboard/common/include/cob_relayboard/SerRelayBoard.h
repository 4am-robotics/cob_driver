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

//#include "stdafx.h"

#include <cob_utilities/IniFile.h>

#include <cob_relayboard/SerialIO.h>

//#include <Neobotix/Drivers/Can/CanCtrlPltfItf.h>
//#include <Neobotix/Drivers/Can/CmdRelaisBoard.h>
#include <canopen_motor/DriveParam.h>
#include <cob_relayboard/Mutex.h>
#include <cob_relayboard/TimeStamp.h>

//-----------------------------------------------

/**
 * Driver class for communication with a Neobotix RelayBoard.
 * Uses RS422 with 420 kBaud.
 */
class SerRelayBoard //: public CanCtrlPltfItf
{
public:

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

	SerRelayBoard();

	~SerRelayBoard();

	// Platform - for function description see interface CanCtrlPltfItf
	bool initPltf();
	bool resetPltf();
	bool shutdownPltf();
	int evalRxBuffer();
	bool isComError();
	bool isDriveError();
	bool isEMStop();
	bool isScannerStop();

	// Drives - for function description see interface CanCtrlPltfItf
	int setWheelVel(int iCanIdent, double dVel, bool bQuickStop);
	int setWheelPosVel(int iCanIdent, double dPos, double dVel, bool bQuickStop);
	void execMotion(int iGroupID) {}
	void sendSynch() {}
	void sendHeartbeat() {}

	void requestDriveStatus();
	int requestMotPosVel(int iCanIdent);
	
	/**
	 * Rotation directions are defined as follows:
	 * Positive wheel angle and velocity is set according to the right hand rule.
	 * The right hands thumb is aligned to the axis from motor to wheel.
	 */
	int getWheelPosVel(	int iCanIdent,
						double* pdAngWheelRad,
						double* pdVelWheelRadS,
						TimeStamp& ts);

	int getWheelDltPosVel(	int iCanIdent,
							double* pdDltAngWheelRad,
							double* pdVelWheelRadS,
							TimeStamp& ts);

	void getStatus(int iCanIdent, int* piStatus, int* piTempCel);
	int disableBrake(int iCanIdent, bool bDisabled);
	int execHoming(int CanIdent); // NOT IMPLEMENTED !!!!
	
	// Relayboard
	int getRelayBoardDigIn();
	int setRelayBoardDigOut(int iChannel, bool bOn);
	int getRelayBoardAnalogIn(int* piAnalogIn);

	// IOBoard - for function description see interface CanCtrlPltfItf
	void requestIOBoardData();
	void getIOBoardJoyValNorm(double* pdJoyXNorm, double* pdJoyYNorm);
	void getIOBoardJoyValWheelMean(double* pdVelWheelLeftMean, double* pdVelWheelRightMean);
	int getIOBoardBattVoltage();
	void requestIOBoardDigIn() { }
	int getIOBoardDigIn();
	int setIOBoardDigOut(int iChannel, bool bVal);
	void requestIOBoardAnalogIn();
	int getIOBoardAnalogIn(int* piAnalogIn);
	void writeIOBoardLCD(int iLine, int iColumn, char cText[]);

	// USBoard - for function description see interface CanCtrlPltfItf
	int startUS(int iChannelActive);
	int stopUS();	
	void requestUSBoardData1To8();
	void requestUSBoardData9To16();
	int getUSBoardData1To8(int* piUSDistMM);
	int getUSBoardData9To16(int* piUSDistMM);
	void requestUSBoardAnalogIn();
	void getUSBoardAnalogIn(int* piAnalogIn);

	// GyroBoard - for function description see interface CanCtrlPltfItf
	void zeroGyro(bool bZeroActive);
	void requestGyroBoardData() { } // done by RelayBoard
	int getGyroBoardAng(double* pdAngRad, double dAcc[]);
	int getGyroBoardDltAng(double* pdAng, double dAcc[]);

	// RadarBoard - for function description see interface CanCtrlPltfItf
	void requestRadarBoardData();
	int getRadarBoardData(double* pdVelMMS);

	// CanClientGeneric - for function description see interface CanCtrlPltfItf
	void addGenericCANListeningId(int id);
	void removeGenericCANListeningId(int id);
	//void getGenericCANMessages(std::vector<CANTimedMessage>& pMessages);
	//void sendGenericCANMessage(CanMsg& message);

protected:

	TimeStamp m_TStampOdo;

	TimeStamp m_CurrTime, m_LastTime;
	
	std::string m_sNumComPort;
		
	DriveParam m_DriveParamLeft, m_DriveParamRight;

	void readConfiguration();
	void sendNetStartCanOpen();

	void txCharArray();
	void rxCharArray();

	void convDataToSendMsg(unsigned char cMsg[]);
	bool convRecMsgToData(unsigned char cMsg[]);

	Mutex m_Mutex;

	double m_dLastPosRight;
	double m_dLastPosLeft;
	unsigned char m_cDebugLeft[4]; 
	unsigned char m_cDebugRight[4];
	unsigned char m_cTextDisplay[20];

	
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

	// Relaisboard
	int m_iRelBoardStatus;
	int m_iChargeCurrent;
	int m_iRelBoardBattVoltage;
	int m_iRelBoardKeyPad;
	int m_iRelBoardIRSensor[4];
	int m_iRelBoardTempSensor;
	
	// IOBoard
	int m_iIOBoardBattVoltage;
	int m_iIOBoardStatus;
	int m_iIOBoardDigIn;
	int m_iIOBoardAnalogIn[8];

	// MotCtrlBoards
	int m_iMotRightStatus;
	int m_iMotLeftStatus;
	int m_iPosMeasMotRightEnc;
	int m_iVelMeasMotRightEncS;
	int m_iPosMeasMotLeftEnc;
	int m_iVelMeasMotLeftEncS;

	// GyroBoard
	int m_iGyroBoardStatus;
	bool m_bGyroBoardZeroGyro;
	int m_iGyroBoardAng;
	int m_iGyroBoardAcc[3];
	
	// RadarBoards
	int m_iRadarBoardStatus;
	int m_iRadarBoardVel[4];

	// USBoard
	int m_iUSBoardStatus;
	int m_iUSBoardSensorData[16];
	int m_iUSBoardAnalogData[4];

	SerialIO m_SerIO;

	bool m_bComInit;
};


//-----------------------------------------------
#endif
