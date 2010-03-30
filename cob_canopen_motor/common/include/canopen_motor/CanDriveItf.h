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
 * ROS stack name: cob3_common
 * ROS package name: canopen_motor
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Feb 2009
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


#ifndef CANDRIVEITF_INCLUDEDEF_H
#define CANDRIVEITF_INCLUDEDEF_H

//-----------------------------------------------
#include <generic_can/CanItf.h>
#include <canopen_motor/DriveParam.h>
#include <canopen_motor/SDOSegmented.h>
//-----------------------------------------------

/**
 * Interface for a drive.
 *  \ingroup DriversCanModul	
 */
class CanDriveItf
{
public:
	/**
	 * Motion type of the controller.
	 */
	enum MotionType
	{
		MOTIONTYPE_VELCTRL,
        MOTIONTYPE_TORQUECTRL,
		MOTIONTYPE_POSCTRL
	};

	/**
	 * Sets the CAN interface.
	 */
	virtual void setCanItf(CanItf* pCanItf) = 0;

	/**
	 * Initializes the driver.
	 * Call this function once after construction.
	 */
	virtual bool init() = 0;
	
	/**
	 * Check if the driver is already initialized.
	 * This is necessary if a drive gets switched off during runtime.
	 * @return true if initialization occured already, false if not.
	 */
	virtual bool isInitialized() = 0;

	/**
	 * Enables the motor.
	 * After calling the drive accepts velocity and position commands.
	 */
	virtual bool start() = 0;

	/**
	 * Disables the motor.
	 * After calling the drive won't accepts velocity and position commands.
	 */
	virtual bool stop() = 0;

	/**
	 * Resets the drive.
	 * The drive changes into the state after initializaton.
	 */
	virtual bool reset() = 0;

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
	 * Inits homing procedure.
	 * Used only in position mode.
	 */
	virtual bool initHoming() = 0;

	/**
	 * Performs homing procedure.
	 * Used only in position mode.
	 */
	virtual bool execHoming() = 0;

	/**
	 * Returns the elapsed time since the last received message.
	 */
	virtual double getTimeToLastMsg() = 0;

	/**
	 * Returns the status of the limit switch needed for homing.
	 * true = limit switch is reached; false = not reached
	 */
	virtual bool getStatusLimitSwitch() = 0;

	/**
	 * Starts the watchdog.
	 * The Harmonica provides watchdog functionality which means the drive stops if the watchdog
	 * becomes active. To keep the watchdog inactive a heartbeat message has to be sent
	 * periodically. The update rate is set to 1s.
	 * The update is is done in function setGearVelRadS().
	 */
	virtual bool startWatchdog(bool bStarted) = 0;

	/**
	 * Evals a received message.
	 * Only messages with fitting identifiers are evaluated.
	 * @param msg message to be evaluated.
	 */
	virtual bool evalReceivedMsg(CanMsg& msg) = 0;

	/**
	 * Evals received messages in OBJECT mode.
	 * The CAN drives have to implement which identifiers they are interested in.
	 * @return true on success, false on failure.
	 */
	virtual bool evalReceivedMsg() = 0;

	/**
	 * Sets required position and veolocity.
	 * Use this function only in position mode.
	 * By calling the function the status is requested, too.
	 */
	virtual void setGearPosVelRadS(double dPosRad, double dVelRadS) = 0;

	/**
	 * Sets the velocity.
	 * By calling the function the status is requested, too.
	 */
	virtual void setGearVelRadS(double dVelRadS) = 0;

	/**
	 * Sets the motion type drive.
	 * The function is not implemented for Harmonica.
	 * The harmonica drive is configured just for velocity mode.
	 */
	virtual bool setTypeMotion(int iType) = 0;

	/**
	 * Returns the position and the velocity of the drive.
	 */
	virtual void getGearPosVelRadS(double* pdAngleGearRad, double* pdVelGearRadS) = 0;

	/**
	 * Returns the change of the position and the velocity.
	 * The given delta position is given since the last call of this function.
	 */
	virtual void getGearDeltaPosVelRadS(double* pdDeltaAngleGearRad, double* pdVelGearRadS) = 0;

	/**
	 * Returns the current position.
	 */
	virtual void getGearPosRad(double* pdPosGearRad) = 0;

	/**
	 * Sets the drive parameter.
	 */
	virtual void setDriveParam(DriveParam driveParam) = 0;

	/**
	 * Returns true if an error has been detected.
	 */
	virtual bool isError() = 0;
	
	/**
	 * Return a bitfield containing information about the pending errors.
	 */
	virtual unsigned int getError() = 0;

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
	virtual void getStatus(int* piStatus, int* piTempCel) = 0;

	/**
	 * Enable the emergency stop.
	 */
	virtual bool setEMStop() = 0;

	/**
	 * Disable the emergency stop.
	 */
	virtual bool resetEMStop() = 0;

	/**
	 * Sends an integer value to the Harmonica using the built in interpreter. cpc-kischa
	 */
	virtual void IntprtSetInt(int iDataLen, char cCmdChar1, char cCmdChar2, int iIndex, int iData) = 0;

    /**
     *Read out Recorder Data from Elmo Controller. cpc-pk
     */
    virtual bool collectRecordedData(int flag, recData ** output) = 0;

	/**
	 * Sends Requests for "active current" to motor via CAN
	 */
	virtual void requestMotorTorque() = 0;
	
	/**
	 * Returns member variable m_MotorCurrent
	 * To update this value call requestMotorCurrent before
	 * and evaluate CAN buffer, or wait one cycle
	 */
	virtual void getMotorTorque(double* dTorqueNm) = 0;
    
    /**
     * Sends command for motor Torque (in Nm)
     */
    virtual void setMotorTorque(double dTorqueNm) = 0;
};


//-----------------------------------------------
#endif

