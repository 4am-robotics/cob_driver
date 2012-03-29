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


#ifndef CANDRIVEITF_INCLUDEDEF_H
#define CANDRIVEITF_INCLUDEDEF_H

//-----------------------------------------------
#include <cob_generic_can/CanItf.h>
#include <cob_canopen_motor/DriveParam.h>
#include <cob_generic_can/stdDef.h>
//-----------------------------------------------

/**
 * Interface for a drive.
 *  \ingroup DriversCanModul	
 */
class CanDriveItf
{
public:

	enum DriveErrors
	{
		DRIVE_NO_ERROR = 0x00000000,
		DRIVE_FEEDBACK_LOSS = 0x00000001,
		DRIVE_PEAK_CURRENT_EXCEEDED = 0x00000002,
		DRIVE_INHIBIT = 0x00000004,
		DRIVE_HALL_ERROR = 0x00000008,
		DRIVE_SPEED_TRACK_ERROR = 0x00000010,
		DRIVE_POS_TRACK_ERROR = 0x00000020,
		DRIVE_INCONS_DATABASE = 0x00000040,
		DRIVE_HEARTBEAT_FAIL = 0x00000080,
		ELOMC_SERVO_DRIVE_FAULT = 0x00000100,
		DRIVE_UNDER_VOLTAGE = 0x00000200,
		DRIVE_OVER_VOLTAGE = 0x00000400,
		DRIVE_SHORT_CIRCUIT = 0x00000800,
		DRIVE_OVER_TEMP = 0x00001000,
		DRIVE_ELECTRICAL_ZERO_NOT_FOUND = 0x00002000,
		DRIVE_SPEED_LIMIT_EXCEEDED = 0x00004000,
		DRIVE_MOTOR_STUCK = 0x00008000,
		DRIVE_POS_LIMIT_EXCEEDED = 0x00010000
	};

	/**
	 * States of the drive.
	 */
	enum StateDrive
	{
		ST_PRE_INITIALIZED,
		ST_OPERATION_ENABLED,
		ST_OPERATION_DISABLED,
		ST_MOTOR_FAILURE
	};

	/**
	 * Motion type of the controller.
	 */
	enum MotionType
	{
		MOTIONTYPE_VELCTRL,
		MOTIONTYPE_POSCTRL
	};

	enum StopType
	{
		NORMAL_STOP = false,
		QUICK_STOP = true
	};


	virtual void setCycleTime(double dt) = 0;


	/**
	 * Sets the CAN interface.
	 */
	virtual void setCanItf(CanItf* pCanItf) = 0;

	/**
	 * Initializes the driver.
	 * Call this function once after construction.
	 */
	virtual bool init(bool bZeroPosCnt) = 0;

	/**
	 * Enables the motor.
	 * After calling the drive accepts velocity and position commands.
	 */
	virtual bool start() = 0;

	/**
	 * Stops the motor after ramping down.
	 */
	virtual void stop() { }

	/**
	 * Sends halt to drive - hard stop e.g. after error
	 */
	virtual void halt() { }

	/**
	 * Shutdowns the motor.
	 */	 
	virtual bool shutdown() = 0;

	/**
	 * Disables the brake.
	 * This function is not implemented for Harmonica,
	 * because brakes are released upon power on and
	 * shut only at emergency stop.
	 */
	virtual bool disableBrake(bool bDisabled) = 0;

	/**
	 * Starts homing procedure.
	 * Used only in position mode.
	 */
	virtual bool prepareHoming() = 0;
	virtual bool initHoming(bool keepDriving = false) = 0;
	virtual bool execHoming() = 0;
	virtual bool isHomingFinished(bool waitTillHomed = true) = 0;
	virtual void exitHoming(double t, bool keepDriving = false) = 0;

	virtual void initWatchdog() = 0;

	virtual void setModuloCount(double min, double max) { }

	/**
	 * Returns the elapsed time since the last received message.
	 */
	virtual double getTimeToLastMsg() = 0;

	/**
	 * Evals a received message.
	 * Only messages with fitting identifiers are evaluated.
	 * @param msg message to be evaluated.
	 */
	virtual bool evalReceivedMsg(CanMsg& msg) = 0;

	/**
	 * Sets the drives position and veolocity.
	 * If using ElmoMC amplifier and dVel = 0 and the drive is in user mode 5 (position control)
	 * the amplifier will use its own trajectory generator to reach dPos.
	 * If dVel != 0 only the velocity is set.
	 * By calling the function the status is requested, too.
	 * @param dPos				commanded position
	 * @param dVel				commanded velocity
	 * @param bBeginMotion		set true if motion should start instantly else use group ID
	 *							of all drives to start motion
	 * @param bUsePosMode		use position mode of amplifier of velocity command = 0
	 */
	virtual bool setWheelPosVel(double dPos, double dVel, bool bBeginMotion, bool bUsePosMode) = 0;

	/**
	 * Sets the drives velocity.
	 * By calling the function the status is also requested.
	 * @param dVel				velocity in [rad/s]
	 * @param bQuickStop		set true if a quick stop is commanded
	 * @param bBeginMotion		set true if motion should start instantly else use group ID
	 *							of all drives to start motion
	 */
	virtual bool setWheelVel(double dVel, bool bQuickStop, bool bBeginMotion) = 0;

	/**
	 * Sets the motion type drive.
	 * The function is not implemented for Harmonica.
	 * The harmonica drive is configured just for velocity mode.
	 */
	virtual bool setTypeMotion(int iType) = 0;

	/**
	 * Returns the angle and the velocity of the drive.
	 * Rotation directions are defined as follows:
	 * Positive wheel angle and velocity is set according to the right hand rule.
	 * The right hands thumb is aligned to the axis from motor to wheel.
	 */
	virtual void getWheelPosVel(double* pdAngWheel, double* pdVelWheel) = 0;

	/**
	 * Returns the change of the position and the velocity.
	 * The given delta position is given since the last call of this function.
	 */
	virtual void getWheelDltPosVel(	double* pdDltAngleWheel, double* pdVelWheel) = 0;

	/**
	 * Returns the current wheel position.
	 */
	virtual void getWheelPos(double* pdPosWheel) = 0;

	/**
	 * Returns true if an error has been detected.
	 */
	virtual bool isError(int* piDigIn) = 0;

	/**
	 * Requests position and velocity.
	 */
	virtual void requestPosVel() = 0;

	/**
	 * Requests status.
	 */
	virtual void requestStatus() = 0;

	/**
	 * Returns the measured temperature. 
	 */
	virtual void getStatus(	int* piStatus,
							int* piCurrentMeasPromille,
							int* piTempCel) = 0;

	/**
	 *
	 */
	virtual bool waitOnDigIn(int iLevel, int iDigIn, double timeout) { return true; }

	/*
	*	if true: motor has reached the current limit
	*/
	virtual bool isCurrentLimit() { return false; }

public:

	DriveParam m_DriveParam;

};

//-----------------------------------------------
#endif
