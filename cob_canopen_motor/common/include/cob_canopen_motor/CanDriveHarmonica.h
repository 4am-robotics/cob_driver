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

#ifndef CANDRIVEHARMONICA_INCLUDEDEF_H
#define CANDRIVEHARMONICA_INCLUDEDEF_H

//-----------------------------------------------

#include <cob_canopen_motor/CanDriveItf.h>
#include <cob_generic_can/stdDef.h>

#include <cob_utilities/TimeStamp.h>

#include <cob_utilities/IniFile.h>

#include <cob_utilities/MathSup.h>

//-----------------------------------------------

// ElmoMC values

// HomeEvent
// 3 high transition of index pulse
// 4 low transition of index pulse
// 5/6 event according to defined FLS switch
// 7/8 event according to defined RLS switch
// 9/10 event according to defined DIN1 switch
// 11/12 event according to defined DIN2 switch ...
// 1000 homing with Neobotix variable block width ring
// 1001 start calibration run for block width ring

// InputPort
// 0 - 5 general purpose input 0 - 5
// 6 main home switch
// 7 aux home switch
// 10 (1024) Forward limit switch
// 11 (2048) Reverse limit switch

/**
 * Driver class for the motor drive of type Harmonica of ElmoMC.
 * \ingroup DriversCanModul	
 */
class CanDriveHarmonica : public CanDriveItf
{
public:

	/**
	 * Internal parameters.
	 */
	struct ParamType
	{
		int iNumRetryOfSend;
		int iDivForRequestStatus;
		double dCANTimeout;
		double dCycleTime;
	};

	/**
	 * Identifier of the CAN messages.
	 */
	struct ParamCANopenID
	{
		int iTxPDO1;
		int iRxPDO1;
		int iTxPDO2;
		int iRxPDO2;
		int iTxPDO3;
		int iRxPDO3;
		int iTxPDO4;
		int iRxPDO4;
		int iTxSDO;
		int iRxSDO;
	};


	/**
	 *
	 */
	CanDriveHarmonica();

	/**
	 *
	 */
	~CanDriveHarmonica() { }

	/**
	 * Sets the CAN interface.
	 */
	void setCanItf(CanItf* pCanItf){ m_pCanCtrl = pCanItf; }

	/*
	 * Sets the control_type:
	 */
	bool setTypeMotion(int iType = 2);
	/**
	 * Sets the time between can calls
	 */
	void setCycleTime(double time);
	/**
	 * Initializes the driver.
	 * Call this function once after construction.
	 */
	bool init(bool bZeroPosCnt);

	/**
	 * Enables the motor.
	 */
	bool start();

	/**
	 * Disables the motor.
	 */
	void stop();

	/**
	 * Shuts down the motor.
	 */
	bool shutdown();

	/**
	 * Disables the brake.
	 * This function is not implemented for Harmonica,
	 * because brakes are released upon power on and
	 * shut only at emergency stop.
	 */
	bool disableBrake(bool bDisabled) { return true; }

	/**
	 * Starts homing procedure. Used only in position mode.
	 */
	bool prepareHoming();

	/**
	 * Initializes ElmoMC amplifier with homing parameters
	 * Returns always true
	 */
	bool initHoming(bool keepDriving);

	/**
	 * home event = 1001: starts block width ring calibration run, returns true
	 * home event = 1000: starts block width ring homing, returns false on timeout
	 * hoem event other: starts limit switch homing, returns true
	 */
	bool execHoming();

	/**
	 * Waits until limit switch of homing is active.
	 * Returns false on timeout.
	 */
	bool isHomingFinished(bool waitTillHomed);

	/**
	 *
	 */
	void exitHoming(double t, bool keepDriving);

	/**
	 *
	 */
	void initWatchdog();

	/**
	 *
	 */
	void setModuloCount(double min, double max);

	/**
	 * Returns the elapsed time since the last received message.
	 */
	double getTimeToLastMsg();

	/**
	 * Evals a received message.
	 * Only messages with fitting identifiers are evaluated.
	 * @param msg message to be evaluated.
	 */
	bool evalReceivedMsg(CanMsg& msg);

	/**
	 * Sets required position and veolocity.
	 * Use this function only in position mode.
	 */
	bool setWheelPosVel(double dPos, double dVel, bool bBeginMotion, bool bUsePosMode);

	/**
	 * Sets the velocity.
	 * By calling the function the status is requested, too.
	 */
	bool setWheelVel(double dVel, bool bQuickStop, bool bBeginMotion);

	/**
	 * Returns the position and the velocity of the drive.
	 */
	void getWheelPosVel(double* pdAngWheel, double* pdVelWheel);

	/**
	 * Returns the change of the position and the velocity.
	 * The given delta position is given since the last call of this function.
	 */
	void getWheelDltPosVel(	double* pdDltAngWheel, double* pdVelWheel);

	/**
	 * Returns the current position.
	 */
	void getWheelPos(double* pdPosWheel);

	/**
	 * Returns true if an error has been detected.
	 */
	bool isError(int* piDigIn);

	/**
	 * Dummy implementation for completing CanDriveItf.
	 */
	void requestPosVel();

	/**
	 * Dummy implementation for completing CanDriveItf.
	 */
	void requestStatus();

	/**
	 * Returns position, velocity and status
	 */
	void getData(	double* pdPosWheelRad, double* pdVelWheelRadS,
					int* piTorqueCtrl, int* piStatus);

	/**
	 * Returns the drives status
	 */
	void getStatus(	int* piStatus, 
					int* piCurrentMeasPromille,
					int* piTempCel);

	/**
	 * @param level			0 = low, 1 = high
	 * @param iNumDigIn		bit number pow 2 of digital input: 1, 2, 4, ...
	 */
	bool waitOnDigIn(int iLevel, int iNumDigIn, double dTimeOutS);

	/**
	 * Sets the CAN identifiers of process and service
	 * data objects
	 */
	void setCANId(int iID);
	
	/**
	 * Sends an integer value to the Harmonica using the built in interpreter.
	 */
	void IntprtSetInt(	int iDataLen,
						char cCmdChar1, char cCmdChar2,
						int iIndex,
						int iData,
						bool bDelay = false);

	/**
	 * Sends a float value to the Harmonica using the built in interpreter.
	 */
	void IntprtSetFloat(int iDataLen,
						char cCmdChar1, char cCmdChar2,
						int iIndex,
						float fData,
						bool bDelay = false);

	/**
	 * Uploads a service data object.
	 */
	void sendSDOUpload(int iObjIndex, int iObjSub);

	/**
	 * Downloads a service data object.
	 */
	void sendSDODownload(int iObjIndex, int iObjSub, int iData, bool bDelay = false);
	
	/**
	 * Evaluats a service data object.
	 */
	void evalSDO(CanMsg& CMsg, int* pIndex, int* pSubindex);
	
	/**
	 * Internal use.
	 */
	int getSDODataInt32(CanMsg& CMsg);

	/**
	*	true: informs that the motor current limit is reached
	*/
	bool isCurrentLimit () { return m_bCurrentLimitOn; };



private:
	int last_pose_incr;
	bool critical_state;

	ParamCANopenID m_ParamCANopen;
	ParamType m_Param;

	CanItf* m_pCanCtrl;
	CanMsg m_CanMsgLast;

	int m_iStatusCtrl;
	int m_iDigIn;

	double m_dWatchdogTime;
	TimeStamp m_SendTime;

	double m_dAngleWheelRadMem;
	double m_dPosWheelMeasRad;
	double m_dLastPos;
	double m_dLastVel;
	double m_dVelWheelMeasRadS;
	int m_iCurrentMeasPromille; // units are promille of rated current

	int m_iMotorState;
	int m_iNewMotorState;
	int m_iCommState;

	bool m_bCurrentLimitOn;

	/**
	 *
	 */
	bool evalStatusRegister(int iStatus);

	/**
	 * If a motor error occured the member variable m_iStatusCtrl is set and a error
	 * is printed. The error enumeration is defined in CanDriveItf.h.
	 */
	void printMotorFailure(int iFailure);

	/**
	 *
	 */
	bool execBlockWidthMeas();

	//LogFile m_LogCAN;
	//LogFile m_LogCANMsg;

	int m_iDivStatus;

};
//-----------------------------------------------
#endif
