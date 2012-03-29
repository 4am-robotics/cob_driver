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

#ifndef NeoCtrlPltfMpo500_INCLUDEDEF_H
#define NeoCtrlPltfMpo500_INCLUDEDEF_H

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
class NeoCtrlPltfMpo500 // : public CanCtrlPltfItf
{
public:

	//--------------------------------- Basic procedures

	/** 
	 * Default constructor.
	 */
	NeoCtrlPltfMpo500();

	/**
	 * Default destructor.
	 */
	~NeoCtrlPltfMpo500();


	//--------------------------------- Hardware Specification

	//--------------------------------- Commands for all nodes on the bus

	/** 
	 * Initializes all CAN nodes of the platfrom and performs homing procedure.
	 * !! The homing routine is hardware-dependent (steering and driving is coupled) !!
	 * !! If you use this code on other hardware -> make sure te remove or adapt homing sequence !! 
	 */
	bool initPltf();

	void sendNetStartCanOpen();
	bool sendSynch();
	/**
	 * Reinitializes the nodes on the bus.
	 * The function might be neccessary after an emergency stop or an hardware failure to reinit drives.
	 */
	bool resetPltf();

	/**
	 * stop all motors and set velocities to 0 
	 */
	bool stopPltf();

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
	 * Sends position to the can node.
	 * Status is requested, too.
	 * @param iCanIdent choose a can node
	 * @param dPosGearRad joint-position in radian
	 * @param dVelGearRadS joint-velocity in radian per second
	 */	
	int setPosGearRad(int iCanIdent, double dPosGearRad, double dVelGearRadS);

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
	void getStatus(int iCanIdent, int* piStatus, int* piCurrentMeasPromille, int* piTempCel);

	/**
	 * Gets the motor torque (calculated from motor active current).
	 * @param iCanIdent choose a can node
	 * @param pdTorqueNm motor-torque in Newtonmeter
	 */
	void getMotorTorque(int iCanIdent, double* pdTorqueNm);



	//--------------------------------- Commands for other nodes

	void timeStep(double dt);


	/**
	 * Parameters charaterising combination of gears and drives
	 */
	struct GearMotorParamType
	{
		int iDriveIdent;
		int iEncIncrPerRevMot;
		double dVelMeasFrqHz;
		double dBeltRatio;
		double dGearRatio;
		int iSign;
		bool bHoming;
		double dHomePos;
		double dHomeVel;
		int iHomeEvent;
		int iHomeCoupleID;
		double iHomeCoupleVel;
		int iHomeDigIn;
		int iHomeTimeOut;
		bool bHomeDSCouple;
		int bHomeDriveId;
		
		double dCurrentToTorque;
		double dCurrentContLimit;
		double dGearEfficiency;
		double dVelMaxEncIncrS;
		double dVelPModeEncIncrS;
		double dAccIncrS2;
		double dDecIncrS2;
		int iTypeEncoder;
		int iCANId;
		bool bUsePosMode;
		bool bEnabled;
	};
	/**
		sets the configuration of this class
	*/
	void readConfiguration(		int typeCan, int baudrateVal,	std::string* sCanDevice, int rate,
					std::vector<DriveParam> driveParamDriveMotor, int iNumMotors,
					std::vector<NeoCtrlPltfMpo500::GearMotorParamType> gearMotDrive,
					bool homeAllAtOnce, std::vector<int> s_control_type, std::vector<int> viMotorID
	);


protected:
	std::vector<int> control_type;

	//--------------------------------- internal functions
	
	/**
	 * Reads configuration of can node and components from Inifile
	 * (should be adapted to use ROS-Parameter file)
	 */
	std::string sIniDirectory;
	std::string sComposed;



	//--------------------------------- Types
	
	/**
	 * Parameters of the class NeoCtrlPltfMpo500.
	 */
	struct ParamType
	{
		// Platform config

		int iHasWheel1DriveMotor;
		int iHasWheel2DriveMotor;
		int iHasWheel3DriveMotor;
		int iHasWheel4DriveMotor;

		int iRadiusWheelMM;
		int iDistSteerAxisToDriveWheelMM;

		double dCanTimeout;
	};


	/**
	 * CAN IDs for Neobotix boards. (Default Values)
	 */
	struct CanNeoIDType
	{
		int DriveNeo_W1Drive_rx_ID;
		int DriveNeo_W1Drive_tx_ID;

		int DriveNeo_W2Drive_rx_ID;
		int DriveNeo_W2Drive_tx_ID;

		int DriveNeo_W3Drive_rx_ID;
		int DriveNeo_W3Drive_tx_ID;

		int DriveNeo_W4Drive_rx_ID;
		int DriveNeo_W4Drive_tx_ID;
	};

	/**
	 * CAN IDs for motor drive Harmonica.
	 * (actually this should be read from inifile)
	 */

	//--------------------------------- Parameter
	ParamType m_Param;
//	CanNeoIDType m_CanNeoIDParam;

	// Prms for all Motor/Gear combos
	std::vector<GearMotorParamType> m_GearMotDrive;

	//--------------------------------- Variables
	CanMsg m_CanMsgRec;
	Mutex m_Mutex;
	bool m_bWatchdogErr;

	//--------------------------------- Components
	// Can-Interface
	CanItf* m_pCanCtrl;

	int m_iNumMotors;
	bool bHomeAllAtOnce;

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

	//can message evaluation
	// other


};


//-----------------------------------------------
#endif
