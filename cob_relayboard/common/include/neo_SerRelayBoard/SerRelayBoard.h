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


#ifndef SerRelayBoard_INCLUDEDEF_H
#define SerRelayBoard_INCLUDEDEF_H

//-----------------------------------------------


#include <neo_SerRelayBoard/SerialIO.h>
#include <neo_SerRelayBoard/Mutex.h>
#include <neo_SerRelayBoard/StrUtil.h>
#include <neo_SerRelayBoard/DriveParam.h>
#include <neo_SerRelayBoard/CanMsg.h>
#include <neo_SerRelayBoard/CmdRelaisBoard.h>
//#include <neo_SerRelayBoard/IniFile.h>
#include <vector>
#include <boost/array.hpp>

//-----------------------------------------------

/**
 * Driver class for communication with a Neobotix RelayBoard.
 * Uses RS422 with 420 kBaud.
 */
class SerRelayBoard
{
public:
	SerRelayBoard();
	//SerRelayBoard(int iTypeLCD, std::string pathToConf="../depreciatedConfig/"); //Deprecated
	~SerRelayBoard();

	// Platform - for function description see interface CanCtrlPltfItf (Old Syntax)
	bool initPltf();
	bool resetPltf();
	bool shutdownPltf();
	int evalRxBuffer();
	bool isComError();
	bool isDriveError();
	bool isEMStop();
	bool isScannerStop();
	void readConfig(	int iTypeLCD,std::string pathToConf, std::string sNumComPort, 	int hasMotorRight, 
				int hasMotorLeft, int hasIOBoard, int hasUSBoard, int hasRadarBoard, int hasGyroBoard, 
				double quickfix1, double quickfix2, DriveParam driveParamLeft, DriveParam driveParamRight
	);
	//new Syntax: 
	int sendRequest(); //sends collected data and requests response (in the Old version this was done by: setWheelVel() or setWheelPosVel();

	//Care-O-Bots Syntax
	bool init();
	bool reset();
	bool shutdown();
	SerRelayBoard(std::string ComPort, int ProtocolVersion = 1);
	
	
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
	int getWheelPosVel(	int iCanIdent, double* pdAngWheelRad, double* pdVelWheelRadS);

	int getWheelDltPosVel(	int iCanIdent, double* pdDltAngWheelRad, double* pdVelWheelRadS);

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
	int getIOBoardDigOut();
	int setIOBoardDigOut(int iChannel, bool bVal);
	void requestIOBoardAnalogIn();
	int getIOBoardAnalogIn(int* piAnalogIn);
	void writeIOBoardLCD(int iLine, int iColumn, const std::string& sText);

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
	int getGyroBoardAngBoost(double* pdAngRad, boost::array<double, 3u>& dAcc);
	int getGyroBoardDltAng(double* pdAng, double dAcc[]);

	// RadarBoard - for function description see interface CanCtrlPltfItf
	void requestRadarBoardData();
	int getRadarBoardData(double* pdVelMMS);

	// CanClientGeneric - for function description see interface CanCtrlPltfItf
	void addGenericCANListeningId(int id);
	void removeGenericCANListeningId(int id);
	void getGenericCANMessages(std::vector<CANTimedMessage>& pMessages);
	void sendGenericCANMessage(CanMsg& message);

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

	/**
	 * CAN node enumeration.
	 */
	enum CANNode
	{
		CANNODE_IOBOARD,
		CANNODE_MOTORRIGHT,
		CANNODE_MOTORLEFT,
		CANNODE_USBOARD
	};

	/**
	 * Motion types.
	 */
	enum MotionType
	{
		MOTIONTYPE_VELCTRL,
		MOTIONTYPE_POSCTRL
	};

	enum TypeLCD
	{
		LCD_20CHAR_TEXT,
		LCD_60CHAR_TEXT
	};

protected:
	std::string m_sNumComPort;

	DriveParam m_DriveParamLeft, m_DriveParamRight;

	Mutex m_Mutex;

	double m_dLastPosRight;
	double m_dLastPosLeft;
	unsigned char m_cDebugLeft[4]; 
	unsigned char m_cDebugRight[4];
	unsigned char m_cTextDisplay[60];

	int m_iNumBytesSend;
	int m_iTypeLCD;

	
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

	void readConfiguration();
	void sendNetStartCanOpen();

	void txCharArray();
	void rxCharArray();

	/**
	 * Type LCD = 20 char text:
	 * 1 byte command
	 * 16 byte data
	 * 20 byte LCD Text
	 * 11 bytes filled with zeros
	 * 2 bytes checksum
	 * -> 50 bytes
	 *
	 * Type LCD = 60 char text:
	 * 1 byte command
	 * 16 byte data
	 * 60 byte LCD Text
	 * 2 bytes checksum
	 * -> 79 bytes
	 */
	void convDataToSendMsg(unsigned char cMsg[]);
	
	/**
	 *
	 */
	bool convRecMsgToData(unsigned char cMsg[]);
private:
	bool autoSendRequest;
	double quickFix[2]; //TODO: quick and dirty odometry fix (rostopic echo /odom)
};


//-----------------------------------------------
#endif
