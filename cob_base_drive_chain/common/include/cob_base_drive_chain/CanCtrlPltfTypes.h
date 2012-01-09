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

#ifndef CANCTRLPLTFTYPES_INCLUDEDEF_H
#define CANCTRLPLTFTYPES_INCLUDEDEF_H



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
	int		iEncOffsetIncr;
	bool	bIsSteer;
	double  dCurrentToTorque;
	double  dCurrMax;
	int 	iHomingDigIn;
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

#endif
