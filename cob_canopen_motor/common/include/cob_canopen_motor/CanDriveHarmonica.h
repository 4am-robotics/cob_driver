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
 * ROS package name: cob_canopen_motor
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Feb 2009
 * ToDo: - Assign Adsress of digital input for homing switch "iHomeDigIn" via parameters (in evalReceived Message, Line 116).
 *       - Homing Event should be defined by a parameterfile and handed to CanDrive... e.g. via the DriveParam.h (in inithoming, Line 531).
 *		 - Check whether "requestStatus" can/should be done in the class implementing the component
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


#ifndef CANDRIVEHARMONICA_INCLUDEDEF_H
#define CANDRIVEHARMONICA_INCLUDEDEF_H

//-----------------------------------------------
#include <cob_canopen_motor/CanDriveItf.h>
#include <cob_utilities/TimeStamp.h>

#include <cob_canopen_motor/SDOSegmented.h>
#include <cob_canopen_motor/ElmoRecorder.h>
//-----------------------------------------------

/**
 * Driver class for the motor drive of type Harmonica.
 * \ingroup DriversCanModul	
 */
class CanDriveHarmonica : public CanDriveItf
{
public:
	// ------------------------- Types
	/**
	 * Internal parameters.
	 */
	struct ParamType
	{
		int iNumRetryOfSend;
		int iDivForRequestStatus;
		double dCanTimeout;	
	};

	/**
	 * States of the drive.
	 */
	enum StateHarmonica
	{
		ST_PRE_INITIALIZED,
		ST_OPERATION_ENABLED,
		ST_OPERATION_DISABLED,
		ST_MOTOR_FAILURE
	};

	/**
	 * Identifier of the CAN messages.
	 */
	struct ParamCanOpenType
	{
		int iTxPDO1;
		int iTxPDO2;
		int iRxPDO2;
		int iTxSDO;
		int iRxSDO;
	};

	// ------------------------- Interface
	/**
	 * Sets the CAN interface.
	 */
	void setCanItf(CanItf* pCanItf){ m_pCanCtrl = pCanItf; }

	/**
	 * Initializes the driver.
	 * Call this function once after construction.
	 */
	bool init();

	/**
	 * Check if the driver is already initialized.
	 * This is necessary if a drive gets switched off during runtime.
	 * @return true if initialization occured already, false if not.
	 */
	bool isInitialized() { return m_bIsInitialized; }
			
	/**
	 * Enables the motor.
	 * After calling the drive accepts velocity and position commands.
	 */
	bool start();

	/**
	 * Disables the motor.
	 * After calling the drive won't accepts velocity and position commands.
	 */
	bool stop();
	/**
	 * Resets the drive.
	 * The drive changes into the state after initializaton.
	 */
	bool reset();

	/**
	 * Shutdown the motor.
	 */	 
	bool shutdown();

	/**
	 * Disables the brake.
	 * This function is not implemented for Harmonica,
	 * because brakes are released upon power on and
	 * shut only at emergency stop.
	 */
	bool disableBrake(bool bDisabled);

	/**
	 * Inits homing procedure.
	 */
	bool initHoming();

	/**
	 * Performs homing procedure.
	 */
	bool execHoming();
	
	/**
	 * Performs homing procedure
	 * Drives wheel in neutral Position for Startup.
	 */
	bool endHoming();

	/**
	 * Returns the elapsed time since the last received message.
	 */
	double getTimeToLastMsg();

	/**
	 * Returns the status of the limit switch needed for homing.
	 * true = limit switch is reached; false = not reached
	 */
	bool getStatusLimitSwitch();

	/**
	 * Starts the watchdog.
	 * The Harmonica provides watchdog functionality which means the drive stops if the watchdog
	 * becomes active. To keep the watchdog inactive a heartbeat message has to be sent
	 * periodically. The update rate is set to 1s.
	 * The update is done in function setGearVelRadS().
	 */
	bool startWatchdog(bool bStarted);

	/**
	 * Evals a received message.
	 * Only messages with fitting identifiers are evaluated.
	 * @param msg message to be evaluated.
	 */
	bool evalReceivedMsg(CanMsg& msg);

	/**
	 * Evals received messages in OBJECT mode.
	 * @todo To be implemented!
	 */
	bool evalReceivedMsg() { return true; }


	/**
	 * Sets required position and veolocity.
	 * Use this function only in position mode.
	 */
	void setGearPosVelRadS(double dPosRad, double dVelRadS);

	/**
	 * Sets the velocity.
	 * By calling the function the status is requested, too.
	 */
	void setGearVelRadS(double dVelEncRadS);

	/**
	 * Sets the motion type drive.
	 */
	bool setTypeMotion(int iType);

	/**
	 * Returns the position and the velocity of the drive.
	 */
	void getGearPosVelRadS(double* pdAngleGearRad, double* pdVelGearRadS);

	/**
	 * Returns the change of the position and the velocity.
	 * The given delta position is given since the last call of this function.
	 */
	void getGearDeltaPosVelRadS(double* pdDeltaAngleGearRad, double* pdVelGearRadS);

	/**
	 * Returns the current position.
	 */
	void getGearPosRad(double* pdPosGearRad);

	/**
	 * Sets the drive parameter.
	 */
	void setDriveParam(DriveParam driveParam) { m_DriveParam = driveParam; }

	/**
	 * Returns true if an error has been detected.
	 */
	bool isError();
	
	/**
	 * Return a bitfield containing information about the pending errors.
	 */
	unsigned int getError() { return 0; }

	// ------ Dummy implementations for completing the interface

	/**
	 * Dummy implementation for completing CanDriveItf.
	 */
	void requestPosVel();

	/**
	 * Requests status :) checks whether motor is operational, switched off or in error state.
	 * If motor is in which error state it checks which error occured
	 */
	void requestStatus();

	/**
	 * Dummy implementation for completing CanDriveItf.
	 */
	void getStatus(int* piStatus, int* piTempCel) { *piStatus = 0; *piTempCel = 0; }


	// ------------------------- Interface: Harmonica specific
	/**
	 * Default constructor.
	 */
	CanDriveHarmonica();

	/**
	 * Returns some received values from the drive.
	 * @deprecated use the other functions instead.
	 * @param pdPosGearRad position of the drive
	 * @param pdVelGearRadS velocity of the drive
	 * @param piTorqueCtrl torque 
	 * @param piStatusCtrl
	 */
	void getData(double* pdPosGearRad, double* pdVelGearRadS,
		int* piTorqueCtrl, int* piStatusCtrl);

	/**
	 * Sets the CAN identifiers of the drive node.
	 * @param iTxPDO1 first transmit process data object
	 * @param iTxPDO2 second transmit process data object
	 * @param iRxPDO2 second receive process data object
	 * @param iTxSDO transmit service data object
	 * @param iRxSDO receive service data object
	 */
	void setCanOpenParam( int iTxPDO1, int iTxPDO2, int iRxPDO2, int iTxSDO, int iRxSDO);
	
	/**
	 * Sends an integer value to the Harmonica using the built in interpreter.
	 */
	void IntprtSetInt(int iDataLen, char cCmdChar1, char cCmdChar2, int iIndex, int iData);
	
	/**
	 * Sends a heartbeat to the CAN-network to keep all listening watchdogs sleeping
	 */
	void sendHeartbeat();
	

	bool setEMStop() {
		std::cout << "The function setEMStop() is not implemented!!!" << std::endl;
		return false;
	}
	
	bool resetEMStop() {
		std::cout << "The function resetEMStop() is not implemented!!!" << std::endl;
		return false;
	}

    /**
	 * Sets MotorTorque in Nm
	 * By sending this command the status is requested, too
	 */	
	void setMotorTorque(double dTorqueNm);

	/**
	 * Sends Requests for "active current" to motor via CAN
     *  To read Motor current perform the following:
     *  1.) request motor current
     *          m_pW1DriveMotor->requestMotorTorque();
     *  2.) evaluate Can buffer to read motor current and decode it
	 *		    evalCanBuffer();
	 */
	void requestMotorTorque();
	
	/**
	 * Return member variable m_MotorCurrent
	 * To update this value call requestMotorCurrent at first
	 */
	void getMotorTorque(double* dTorqueNm);

	/**
	 * Provides several functions for drive information recording purposes using the built in ElmoRecorder, which allows to record drive information at a high frequency. 
	 * @param iFlag To keep the interface slight, use iParam to command the recorder:
	 * 0: Configure the Recorder to record the sources Main Speed(1), Main position(2), Active current(10), Speed command(16). With iParam = iRecordingGap you specify every which time quantum (4*90usec) a new data point (of 1024 points in total) is recorded; 
	 * 1: Query Upload of recorded source (1=Main Speed, 2=Main position, 10=Active Current, 16=Speed command) with iParam and log data to file sParam = file prefix. Filename is extended with _MotorNumber_RecordedSource.log
	 * 2: Request status of ongoing readout process
	 * 99: Abort and clear current SDO readout process
	 * @return 0: Success, 1: Recorder hasn't been configured yet, 2: data collection still in progress
	 *
	*/
	int setRecorder(int iFlag, int iParam = 0, std::string sParam = "/home/MyLog_");
	
	
	//--------------------------
	//CanDriveHarmonica specific functions (not from CanDriveItf)
	//--------------------------
	/**
	 * Sends a float value to the Harmonica using the built in interpreter.
	 */
	void IntprtSetFloat(int iDataLen, char cCmdChar1, char cCmdChar2, int iIndex, float fData);

	/**
	 * CANopen: Uploads a service data object (device to master). (in expedited transfer mode, means in only one message)
	 */
	void sendSDOUpload(int iObjIndex, int iObjSub);
	
    /**
	 * CANopen: This protocol cancels an active segmented transmission due to the given Error Code
	 */
    void sendSDOAbort(int iObjIndex, int iObjSubIndex, unsigned int iErrorCode);

	/**
	 * CANopen: Downloads a service data object (master to device). (in expedited transfer mode, means in only one message)
	 */
	void sendSDODownload(int iObjIndex, int iObjSub, int iData);
	
	/**
	 * CANopen: Evaluates a service data object and gives back object and sub-object ID
	 */
	void evalSDO(CanMsg& CMsg, int* pIndex, int* pSubindex);
	
	/**
	 * Internal use.
	 */
	int getSDODataInt32(CanMsg& CMsg);
	
    
protected:
	// ------------------------- Parameters
	ParamCanOpenType m_ParamCanOpen;
	DriveParam m_DriveParam;
	bool m_bLimitSwitchEnabled;
	ParamType m_Param;

	// ------------------------- Variables
	CanItf* m_pCanCtrl;
	CanMsg m_CanMsgLast;

	ElmoRecorder* ElmoRec;

	int m_iTypeMotion;
	int m_iStatusCtrl;
	int	m_iTorqueCtrl;

	TimeStamp m_CurrentTime;
	TimeStamp m_WatchdogTime;
	TimeStamp m_VelCalcTime;
	TimeStamp m_FailureStartTime;
	TimeStamp m_SendTime;
	TimeStamp m_StartTime;

	double m_dAngleGearRadMem;
	double m_dVelGearMeasRadS;
	double m_dPosGearMeasRad;

	bool m_bLimSwLeft;
	bool m_bLimSwRight;

	double m_dOldPos;

	std::string m_sErrorMessage;

	int m_iMotorState;
	int m_iNewMotorState;

	int m_iCountRequestDiv;

	bool m_bCurrentLimitOn;

	int m_iNumAttempsRecFail;

	bool m_bOutputOfFailure;

	bool m_bIsInitialized;

	double m_dMotorCurr;

	bool m_bWatchdogActive;

	segData seg_Data;


	// ------------------------- Member functions
	double estimVel(double dPos);

	bool evalStatusRegister(int iStatus);
	void evalMotorFailure(int iFailure);
	
	int m_iPartnerDriveRatio;
	int m_iDistSteerAxisToDriveWheelMM;

	bool isBitSet(int iVal, int iNrBit)
	{
		if( (iVal & (1 << iNrBit)) == 0)
			return false;
		else
			return true;
	}

	/**
	 * CANopen: Answer a data mesage during a segmented SDO transfer including a toggling bit. Current toggle bit is stored in the SDOSegmented container.
	 * Function is called, when any SDO transfer segment is received (by receivedSDODataSegment)
	 */
	void sendSDOUploadSegmentConfirmation(bool toggleBit);
    
    /**
	 * CANopen: Segment data is stored to the SDOSegmented container.
	 * Function is called, when a segment during a segmented SDO transfer is received (by evalReceivedMsg). It analyzes the SDO transfer header to see, if the transfer is finished. If it's not finished, sendSDOUploadSegmentConfirmation is called to confirm the receive of the current segment and request the next one. 
	 * -Currently only used for Elmo Recorder read-out
	 * @see evalReceivedMsg()
	 * @see sendSDOUploadSegmentConfirmation()
	 * @see finishedSDOSegmentedTransfer()
	 */
	int receivedSDODataSegment(CanMsg& msg);

	/**
	 * CANopen: Evaluates a service data object and gives back object and sub-object ID.
	 * Function is called, when a SDO initialize transfer segment is received (by evalReceivedMsg)
	 * @see evalReceivedMsg()
	 */
	int receivedSDOSegmentedInitiation(CanMsg& msg);
    
    /**
	 * CANopen: Function is called by evalReceivedMsg when the current segmented SDO transfer is cancelled with an error code.
	 * @see evalReceivedMsg()
	 */
	void receivedSDOTransferAbort(unsigned int iErrorCode);
    
    /**
	 * CANopen: Give the collected data of a finished segmented SDO transfer to an appropriate (depending on the current object ID) processing function.
	 * Function is called by receivedSDODataSegment, when the transfer is finished. 
	 * Currently, only Elmo Recorder data is uploaded segmented and is processed here.
	 * @see receivedSDODataSegment()
	 */
	void finishedSDOSegmentedTransfer();

};
//-----------------------------------------------
#endif
