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
 * ROS stack name: cob_drivers
 * ROS package name: cob_base_drive_chain
 * Description: This is a sample implementation of a can-bus with several nodes. In this case it implements the drive-chain of the Care-O-bot3 mobile base. yet, this can be used as template for using the generic_can and canopen_motor packages to implement arbitrary can-setups.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Feb 2010
 * ToDo: - Check whether motor status request in "setVelGearRadS" "setMotorTorque" make sense (maybe remove in "CanDriveHarmonica").
 *		 - move implementational details (can cmds) of configureElmoRecorder to CanDriveHarmonica (check whether CanDriveItf has to be adapted then)
 *		 - Check: what is the iRecordingGap, what is its unit
 *		 - Remove Mutex.h search for a Boost lib
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

#ifndef CANCTRLPLTFCOB3_INCLUDEDEF_H
#define CANCTRLPLTFCOB3_INCLUDEDEF_H

//-----------------------------------------------

// general includes

// Headers provided by other cob-packages
#include <cob_canopen_motor/CanDriveItf.h>
#include <cob_canopen_motor/CanDriveHarmonica.h>
#include <cob_generic_can/CanItf.h>

// Headers provided by cob-packages which should be avoided/removed
#include <cob_utilities/IniFile.h>
#include <cob_utilities/Mutex.h>

// remove (not supported)
//#include "stdafx.h"


//-----------------------------------------------

/**
 * Represents all CAN components on an arbitrary canbus.
 */
class CanCtrlPltfCOb3 // : public CanCtrlPltfItf
{
public:

	//--------------------------------- Basic procedures

	/** 
	 * Default constructor.
	 */
	CanCtrlPltfCOb3(std::string iniDirectory);

	/**
	 * Default destructor.
	 */
	~CanCtrlPltfCOb3();


	//--------------------------------- Hardware Specification

	/**
	 * Specify Cannodes (Identifiers to send velocities to one specific motor)
	 * This has to be adapted to the hardware-setup!
	 */
	enum MotorCANNode
	{
		CANNODE_WHEEL1DRIVEMOTOR,
		CANNODE_WHEEL1STEERMOTOR,
		CANNODE_WHEEL2DRIVEMOTOR,
		CANNODE_WHEEL2STEERMOTOR,
		CANNODE_WHEEL3DRIVEMOTOR,
		CANNODE_WHEEL3STEERMOTOR,
		CANNODE_WHEEL4DRIVEMOTOR,
		CANNODE_WHEEL4STEERMOTOR
	};


	//--------------------------------- Commands for all nodes on the bus

	/** 
	 * Initializes all CAN nodes of the platfrom and performs homing procedure.
	 * !! The homing routine is hardware-dependent (steering and driving is coupled) !!
	 * !! If you use this code on other hardware -> make sure te remove or adapt homing sequence !! 
	 */
	bool initPltf();

	/**
	 * Reinitializes the nodes on the bus.
	 * The function might be neccessary after an emergency stop or an hardware failure to reinit drives.
	 */
	bool resetPltf();

	/**
	 * Signs an error of the platform.
	 * @return true if there is an error.
	 */
	bool isPltfError();

	/**
	 * Shutdown of the platform.
	 * Disables motors, enables brake and disconnects.
	 */
	bool shutdownPltf();

	/**
	 * Starts the watchdog of the CAN components.
	 */
	bool startWatchdog(bool bStarted);

	/**
	 * Triggers evaluation of the can-buffer.
	 */
	int evalCanBuffer();


	//--------------------------------- Commands specific for motor controller nodes

	/**
	 * Sends veolocities to the can node.
	 * Status is requested, too.
	 * @param iCanIdent choose a can node
	 * @param dVelGearRadS joint-velocity in radian per second
	 */	
	int setVelGearRadS(int iCanIdent, double dVelGearRadS);

	/**
	 * Sends torques to the can node.
	 * Status is requested, too.
	 * @param iCanIdent choose a can node
	 * @param dTorqueNM motor-torque in Newtonmeter
	 */	
	void setMotorTorque(int iCanIdent, double dTorqueNm);

	/**
	 * Requests the status of the drive.
	 * Status-Msg includes whether motor is in error-state
	 * (with source, e.g. undercurrent, overheated)
	 * or operational (enabled/disabled, at its limits, ...)
	 */
	void requestDriveStatus();

	/**
	 * Requests position and velocity of the drive.
	 * (This is not implemented for CanDriveHarmonica.
	 * sending an Velocity command to an ELMO Harmonica-Ctrl
	 * triggers a aswer transmitting the current velocity)
	 * @param iCanIdent choose a can node
	 */
	int requestMotPosVel(int iCanIdent);

	/**
	 * Requests motor-torque (in fact active current) of the drive.
	 * @param iCanIdent choose a can node
	 */
	void requestMotorTorque();

	/**
	 * Gets the position and velocity.
	 * @param iCanIdent choose a can node
	 * @param pdAngleGearRad joint-position in radian
	 * @param pdVelGearRadS joint-velocity in radian per second
	 */
	int getGearPosVelRadS(int iCanIdent, double* pdAngleGearRad, double* pdVelGearRadS);

	/**
	 * Gets the delta joint-angle since the last call and the velocity.
	 * @param iCanIdent choose a can node
	 * @param pdDeltaAngleGearRad delta joint-position since the last call in radian
	 * @param pdVelGearRadS joint-velocity in radian per second
	 */
	int getGearDeltaPosVelRadS(int iCanIdent, double* pdDeltaAngleGearRad, double* pdVelGearRadS);

	/**
	 * Gets the status and temperature in degree celcius.
	 * (Not implemented for CanDriveHarmonica)
	 * @param iCanIdent choose a CANNode enumatraion
	 */
	void getStatus(int iCanIdent, int* piStatus, int* piTempCel);

	/**
	 * Gets the motor torque (calculated from motor active current).
	 * @param iCanIdent choose a can node
	 * @param pdTorqueNm motor-torque in Newtonmeter
	 */
	void getMotorTorque(int iCanIdent, double* pdTorqueNm);



	//--------------------------------- Commands specific for a certain motor controller
	// have to be implemented here, to keep the CanDriveItf generic

	/**
	 * Provides several functions for drive information recording purposes using the built in ElmoRecorder, which allows to record drive information at a high frequency. 
	 * @param iFlag To keep the interface slight, use iParam to command the recorder:
	 * 0: Configure the Recorder to record the sources Main Speed(1), Main position(2), Active current(10), Speed command(16). With iParam = iRecordingGap you specify every which time quantum (4*90usec) a new data point (of 1024 points in total) is recorded; 
	 * 1: Query Upload of recorded source (1=Main Speed, 2=Main position, 10=Active Current, 16=Speed command) with iParam and log data to file sParam = file prefix. Filename is extended with _MotorNumber_RecordedSource.log
	 * 99: Abort and clear current SDO readout process
	 * @return -1: Unknown flag set; 0: Success; 1: Recorder hasn't been configured yet; 2: data collection still in progress
	 *
	*/
	int ElmoRecordings(int iFlag, int iParam, std::string sString);

	//--------------------------------- Commands for other nodes




protected:

	//--------------------------------- internal functions
	
	/**
	 * Reads configuration of can node and components from Inifile
	 * (should be adapted to use ROS-Parameter file)
	 */
	std::string sIniDirectory;
	std::string sComposed;
	void readConfiguration();

	/**
	 * Starts up can node
	 */
	void sendNetStartCanOpen();


	//--------------------------------- Types
	
	/**
	 * Parameters of the class CanCtrlPltfCOb3.
	 */
	struct ParamType
	{
		// Platform config

		int iHasWheel1DriveMotor;
		int iHasWheel1SteerMotor;
		int iHasWheel2DriveMotor;
		int iHasWheel2SteerMotor;
		int iHasWheel3DriveMotor;
		int iHasWheel3SteerMotor;
		int iHasWheel4DriveMotor;
		int iHasWheel4SteerMotor;

		double dWheel1SteerDriveCoupling;
		double dWheel2SteerDriveCoupling;
		double dWheel3SteerDriveCoupling;
		double dWheel4SteerDriveCoupling;

		int iRadiusWheelMM;
		int iDistSteerAxisToDriveWheelMM;

		int iHasRelayBoard;
		int iHasIOBoard;
		int iHasUSBoard;
		int iHasGyroBoard;
		int iHasRadarBoard;

		double dCanTimeout;
	};

	/**
	 * Parameters charaterising combination of gears and drives
	 */
	struct GearMotorParamType
	{
		int		iEncIncrPerRevMot;
		double	dVelMeasFrqHz;
		double	dGearRatio;
		double	dBeltRatio;
		int		iSign;
		double	dVelMaxEncIncrS;
		double	dAccIncrS2;
		double	dDecIncrS2;
		double	dScaleToMM;
		int	iEncOffsetIncr;
		bool	bIsSteer;
		double  dCurrentToTorque;
		double  dCurrMax;
	};

	/**
	 * CAN IDs for Neobotix boards. (Default Values)
	 */
	struct CanNeoIDType
	{
		int IOBoard_rx_ID;
		int IOBoard_tx_ID;

		int DriveNeo_W1Drive_rx_ID;
		int DriveNeo_W1Drive_tx_ID;
		int DriveNeo_W1Steer_rx_ID;
		int DriveNeo_W1Steer_tx_ID;

		int DriveNeo_W2Drive_rx_ID;
		int DriveNeo_W2Drive_tx_ID;
		int DriveNeo_W2Steer_rx_ID;
		int DriveNeo_W2Steer_tx_ID;

		int DriveNeo_W3Drive_rx_ID;
		int DriveNeo_W3Drive_tx_ID;
		int DriveNeo_W3Steer_rx_ID;
		int DriveNeo_W3Steer_tx_ID;

		int DriveNeo_W4Drive_rx_ID;
		int DriveNeo_W4Drive_tx_ID;
		int DriveNeo_W4Steer_rx_ID;
		int DriveNeo_W4Steer_tx_ID;

		int USBoard_rx_ID;
		int USBoard_tx_ID;
		int GyroBoard_rx_ID;
		int GyroBoard_tx_ID;
		int RadarBoard_rx_ID;
		int RadarBoard_tx_ID;
	};

	/**
	 * CAN IDs for motor drive Harmonica.
	 * (actually this should be read from inifile)
	 */
	struct CanOpenIDType
	{	
		// Wheel 1
		// can adresse motor 1
		int TxPDO1_W1Drive;
		int TxPDO2_W1Drive;
		int RxPDO2_W1Drive;
		int TxSDO_W1Drive;
		int RxSDO_W1Drive;
		// can adresse motor 2
		int TxPDO1_W1Steer;
		int TxPDO2_W1Steer;
		int RxPDO2_W1Steer;
		int TxSDO_W1Steer;
		int RxSDO_W1Steer;

		// Wheel 2
		// can adresse motor 7
		int TxPDO1_W2Drive;
		int TxPDO2_W2Drive;
		int RxPDO2_W2Drive;
		int TxSDO_W2Drive;
		int RxSDO_W2Drive;
		// can adresse motor 8
		int TxPDO1_W2Steer;
		int TxPDO2_W2Steer;
		int RxPDO2_W2Steer;
		int TxSDO_W2Steer;
		int RxSDO_W2Steer;	
		
		// Wheel 3
		// can adresse motor 5
		int TxPDO1_W3Drive;
		int TxPDO2_W3Drive;
		int RxPDO2_W3Drive;
		int TxSDO_W3Drive;
		int RxSDO_W3Drive;
		// can adresse motor 6
		int TxPDO1_W3Steer;
		int TxPDO2_W3Steer;
		int RxPDO2_W3Steer;
		int TxSDO_W3Steer;
		int RxSDO_W3Steer;
		
		// Wheel 4
		// can adresse motor 3
		int TxPDO1_W4Drive;
		int TxPDO2_W4Drive;
		int RxPDO2_W4Drive;
		int TxSDO_W4Drive;
		int RxSDO_W4Drive;
		// can adresse motor 4
		int TxPDO1_W4Steer;
		int TxPDO2_W4Steer;
		int RxPDO2_W4Steer;
		int TxSDO_W4Steer;
		int RxSDO_W4Steer;	
	};

	//--------------------------------- Parameter
	ParamType m_Param;
//	CanNeoIDType m_CanNeoIDParam;
	CanOpenIDType m_CanOpenIDParam;

	// Prms for all Motor/Gear combos
	GearMotorParamType m_GearMotDrive1;
	GearMotorParamType m_GearMotDrive2;
	GearMotorParamType m_GearMotDrive3;
	GearMotorParamType m_GearMotDrive4;	
	GearMotorParamType m_GearMotSteer1;
	GearMotorParamType m_GearMotSteer2;
	GearMotorParamType m_GearMotSteer3;
	GearMotorParamType m_GearMotSteer4;

	//--------------------------------- Variables
	CanMsg m_CanMsgRec;
	Mutex m_Mutex;
	bool m_bWatchdogErr;

	//--------------------------------- Components
	// Can-Interface
	CanItf* m_pCanCtrl;
	IniFile m_IniFile;

	int m_iNumMotors;
	int m_iNumDrives;

	// Motor-Controllers
/*	CanDriveItf* m_pW1DriveMotor;
	CanDriveItf* m_pW1SteerMotor;
	CanDriveItf* m_pW2DriveMotor;
	CanDriveItf* m_pW2SteerMotor;
	CanDriveItf* m_pW3DriveMotor;
	CanDriveItf* m_pW3SteerMotor;
	CanDriveItf* m_pW4DriveMotor;
	CanDriveItf* m_pW4SteerMotor;*/
	// pointer to each motors Can-Itf
	std::vector<CanDriveItf*> m_vpMotor;
	// vector with enums (specifying hardware-structure) -> simplifies cmd-check
	// this has to be adapted in c++ file to your hardware
	std::vector<int> m_viMotorID;

	// other


};


//-----------------------------------------------
#endif
