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
 * Supervised by: Christian Connette
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

#ifndef SerRelayBoard_INCLUDEDEF_H
#define SerRelayBoard_INCLUDEDEF_H

//-----------------------------------------------
#include <cob_relayboard/SerialIO.h>
#include <cob_canopen_motor/DriveParam.h>
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
