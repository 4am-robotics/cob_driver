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
 *	 * Redistributions of source code must retain the above copyright
 *	   notice, this list of conditions and the following disclaimer.
 *	 * Redistributions in binary form must reproduce the above copyright
 *	   notice, this list of conditions and the following disclaimer in the
 *	   documentation and/or other materials provided with the distribution.
 *	 * Neither the name of the Fraunhofer Institute for Manufacturing 
 *	   Engineering and Automation (IPA) nor the names of its
 *	   contributors may be used to endorse or promote products derived from
 *	   this software without specific prior written permission.
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

// general includes
#include <math.h>
#include <unistd.h>

// Headers provided by other cob-packages
#include <cob_generic_can/CanESD.h>
#include <cob_generic_can/CanPeakSys.h>
#include <cob_generic_can/CanPeakSysUSB.h>
#include <cob_base_drive_chain/CanCtrlPltfCOb3.h>

#include <unistd.h>

//-----------------------------------------------

CanCtrlPltfCOb3::CanCtrlPltfCOb3(PlatformParams platform_parameters)
{	
	//sIniDirectory = iniDirectory;
	m_PlatformParams = platform_parameters;

	m_iNumDrives = m_PlatformParams.num_wheels;
	m_iNumMotors = m_iNumDrives * 2;

	if(m_iNumMotors < 2 || m_iNumMotors > 8) {
		m_iNumMotors = 8;
		m_iNumDrives = 4;
	}

	// ------------- first of all set used CanItf
	m_pCanCtrl = NULL;

	// ------------- init hardware-specific vectors and set default values
	m_vpMotor.resize(m_iNumMotors);

	for(int i=0; i<m_iNumMotors; i++)
	{
		m_vpMotor[i] = NULL;
	}

	m_viMotorID.resize(m_iNumMotors);

//	m_viMotorID.resize(8);
	if(m_iNumMotors >= 1)
		m_viMotorID[0] = CANNODE_WHEEL1DRIVEMOTOR;
	if(m_iNumMotors >= 2)
		m_viMotorID[1] = CANNODE_WHEEL1STEERMOTOR;
	if(m_iNumMotors >= 3)
		m_viMotorID[2] = CANNODE_WHEEL2DRIVEMOTOR;
	if(m_iNumMotors >= 4)
		m_viMotorID[3] = CANNODE_WHEEL2STEERMOTOR;
	if(m_iNumMotors >= 5)
		m_viMotorID[4] = CANNODE_WHEEL3DRIVEMOTOR;
	if(m_iNumMotors >= 6)
		m_viMotorID[5] = CANNODE_WHEEL3STEERMOTOR;
	if(m_iNumMotors >= 7)
		m_viMotorID[6] = CANNODE_WHEEL4DRIVEMOTOR;
	if(m_iNumMotors == 8)
		m_viMotorID[7] = CANNODE_WHEEL4STEERMOTOR;

	m_bWatchdogErr = false;
}

//-----------------------------------------------
CanCtrlPltfCOb3::~CanCtrlPltfCOb3()
{

	if (m_pCanCtrl != NULL)
	{
		delete m_pCanCtrl;
	}

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		if (m_vpMotor[i] != NULL)
		{
			delete m_vpMotor[i];
		}
	}

}

//-----------------------------------------------
void CanCtrlPltfCOb3::readConfiguration()
{
	DriveParam DriveParamW1DriveMotor;
	DriveParam DriveParamW1SteerMotor;
	DriveParam DriveParamW2DriveMotor;
	DriveParam DriveParamW2SteerMotor;
	DriveParam DriveParamW3DriveMotor;
	DriveParam DriveParamW3SteerMotor;
	DriveParam DriveParamW4DriveMotor;
	DriveParam DriveParamW4SteerMotor;
	
	std::string strTypeDrive;
	std::string strTypeSteer;
	
	// Select CAN-interface here
	m_pCanCtrl = new CANPeakSysUSB(m_PlatformParams.can_device_params);
	
	// CanOpenId's
	m_CanOpenIDParam = m_PlatformParams.canopen_ids;

	// Drive parameters
	if(m_iNumDrives >= 1)
	{
		DriveParamW1DriveMotor.setParam(
			0,
			m_PlatformParams.drive_param_drive.at(0).iEncIncrPerRevMot,
			m_PlatformParams.drive_param_drive.at(0).dVelMeasFrqHz,
			m_PlatformParams.drive_param_drive.at(0).dBeltRatio,
			m_PlatformParams.drive_param_drive.at(0).dGearRatio,
			m_PlatformParams.drive_param_drive.at(0).iSign,
			m_PlatformParams.drive_param_drive.at(0).dVelMaxEncIncrS,
			m_PlatformParams.drive_param_drive.at(0).dAccIncrS2,
			m_PlatformParams.drive_param_drive.at(0).dDecIncrS2,
			m_PlatformParams.drive_param_drive.at(0).iEncOffsetIncr,
			m_PlatformParams.drive_param_drive.at(0).bIsSteer,
			m_PlatformParams.drive_param_drive.at(0).dCurrentToTorque,
			m_PlatformParams.drive_param_drive.at(0).dCurrMax,
			m_PlatformParams.drive_param_drive.at(0).iHomingDigIn);
	}
	
	if(m_iNumDrives >= 1)
	{
		DriveParamW1SteerMotor.setParam(
			1,
			m_PlatformParams.drive_param_steer.at(0).iEncIncrPerRevMot,
			m_PlatformParams.drive_param_steer.at(0).dVelMeasFrqHz,
			m_PlatformParams.drive_param_steer.at(0).dBeltRatio,
			m_PlatformParams.drive_param_steer.at(0).dGearRatio,
			m_PlatformParams.drive_param_steer.at(0).iSign,
			m_PlatformParams.drive_param_steer.at(0).dVelMaxEncIncrS,
			m_PlatformParams.drive_param_steer.at(0).dAccIncrS2,
			m_PlatformParams.drive_param_steer.at(0).dDecIncrS2,
			m_PlatformParams.drive_param_steer.at(0).iEncOffsetIncr,
			m_PlatformParams.drive_param_steer.at(0).bIsSteer,
			m_PlatformParams.drive_param_steer.at(0).dCurrentToTorque,
			m_PlatformParams.drive_param_steer.at(0).dCurrMax,
			m_PlatformParams.drive_param_steer.at(0).iHomingDigIn);
	}

	if(m_iNumDrives >= 2)
	{
		DriveParamW2DriveMotor.setParam(
			2,
			m_PlatformParams.drive_param_drive.at(1).iEncIncrPerRevMot,
			m_PlatformParams.drive_param_drive.at(1).dVelMeasFrqHz,
			m_PlatformParams.drive_param_drive.at(1).dBeltRatio,
			m_PlatformParams.drive_param_drive.at(1).dGearRatio,
			m_PlatformParams.drive_param_drive.at(1).iSign,
			m_PlatformParams.drive_param_drive.at(1).dVelMaxEncIncrS,
			m_PlatformParams.drive_param_drive.at(1).dAccIncrS2,
			m_PlatformParams.drive_param_drive.at(1).dDecIncrS2,
			m_PlatformParams.drive_param_drive.at(1).iEncOffsetIncr,
			m_PlatformParams.drive_param_drive.at(1).bIsSteer,
			m_PlatformParams.drive_param_drive.at(1).dCurrentToTorque,
			m_PlatformParams.drive_param_drive.at(1).dCurrMax,
			m_PlatformParams.drive_param_drive.at(1).iHomingDigIn);
	}
	
	if(m_iNumDrives >= 2)
	{
		DriveParamW2SteerMotor.setParam(
			3,
			m_PlatformParams.drive_param_steer.at(1).iEncIncrPerRevMot,
			m_PlatformParams.drive_param_steer.at(1).dVelMeasFrqHz,
			m_PlatformParams.drive_param_steer.at(1).dBeltRatio,
			m_PlatformParams.drive_param_steer.at(1).dGearRatio,
			m_PlatformParams.drive_param_steer.at(1).iSign,
			m_PlatformParams.drive_param_steer.at(1).dVelMaxEncIncrS,
			m_PlatformParams.drive_param_steer.at(1).dAccIncrS2,
			m_PlatformParams.drive_param_steer.at(1).dDecIncrS2,
			m_PlatformParams.drive_param_steer.at(1).iEncOffsetIncr,
			m_PlatformParams.drive_param_steer.at(1).bIsSteer,
			m_PlatformParams.drive_param_steer.at(1).dCurrentToTorque,
			m_PlatformParams.drive_param_steer.at(1).dCurrMax,
			m_PlatformParams.drive_param_steer.at(1).iHomingDigIn);
	}

	if(m_iNumDrives >= 3)
	{
		DriveParamW3DriveMotor.setParam(
			4,
			m_PlatformParams.drive_param_drive.at(2).iEncIncrPerRevMot,
			m_PlatformParams.drive_param_drive.at(2).dVelMeasFrqHz,
			m_PlatformParams.drive_param_drive.at(2).dBeltRatio,
			m_PlatformParams.drive_param_drive.at(2).dGearRatio,
			m_PlatformParams.drive_param_drive.at(2).iSign,
			m_PlatformParams.drive_param_drive.at(2).dVelMaxEncIncrS,
			m_PlatformParams.drive_param_drive.at(2).dAccIncrS2,
			m_PlatformParams.drive_param_drive.at(2).dDecIncrS2,
			m_PlatformParams.drive_param_drive.at(2).iEncOffsetIncr,
			m_PlatformParams.drive_param_drive.at(2).bIsSteer,
			m_PlatformParams.drive_param_drive.at(2).dCurrentToTorque,
			m_PlatformParams.drive_param_drive.at(2).dCurrMax,
			m_PlatformParams.drive_param_drive.at(2).iHomingDigIn);
	}
	
	if(m_iNumDrives >= 3)
	{
		DriveParamW3SteerMotor.setParam(
			5,
			m_PlatformParams.drive_param_steer.at(2).iEncIncrPerRevMot,
			m_PlatformParams.drive_param_steer.at(2).dVelMeasFrqHz,
			m_PlatformParams.drive_param_steer.at(2).dBeltRatio,
			m_PlatformParams.drive_param_steer.at(2).dGearRatio,
			m_PlatformParams.drive_param_steer.at(2).iSign,
			m_PlatformParams.drive_param_steer.at(2).dVelMaxEncIncrS,
			m_PlatformParams.drive_param_steer.at(2).dAccIncrS2,
			m_PlatformParams.drive_param_steer.at(2).dDecIncrS2,
			m_PlatformParams.drive_param_steer.at(2).iEncOffsetIncr,
			m_PlatformParams.drive_param_steer.at(2).bIsSteer,
			m_PlatformParams.drive_param_steer.at(2).dCurrentToTorque,
			m_PlatformParams.drive_param_steer.at(2).dCurrMax,
			m_PlatformParams.drive_param_steer.at(2).iHomingDigIn);
	}

	if(m_iNumDrives == 4)
	{
		DriveParamW4DriveMotor.setParam(
			6,
			m_PlatformParams.drive_param_drive.at(3).iEncIncrPerRevMot,
			m_PlatformParams.drive_param_drive.at(3).dVelMeasFrqHz,
			m_PlatformParams.drive_param_drive.at(3).dBeltRatio,
			m_PlatformParams.drive_param_drive.at(3).dGearRatio,
			m_PlatformParams.drive_param_drive.at(3).iSign,
			m_PlatformParams.drive_param_drive.at(3).dVelMaxEncIncrS,
			m_PlatformParams.drive_param_drive.at(3).dAccIncrS2,
			m_PlatformParams.drive_param_drive.at(3).dDecIncrS2,
			m_PlatformParams.drive_param_drive.at(3).iEncOffsetIncr,
			m_PlatformParams.drive_param_drive.at(3).bIsSteer,
			m_PlatformParams.drive_param_drive.at(3).dCurrentToTorque,
			m_PlatformParams.drive_param_drive.at(3).dCurrMax,
			m_PlatformParams.drive_param_drive.at(3).iHomingDigIn);
	}
	
	if(m_iNumDrives == 4)
	{
		DriveParamW4SteerMotor.setParam(
			7,
			m_PlatformParams.drive_param_steer.at(3).iEncIncrPerRevMot,
			m_PlatformParams.drive_param_steer.at(3).dVelMeasFrqHz,
			m_PlatformParams.drive_param_steer.at(3).dBeltRatio,
			m_PlatformParams.drive_param_steer.at(3).dGearRatio,
			m_PlatformParams.drive_param_steer.at(3).iSign,
			m_PlatformParams.drive_param_steer.at(3).dVelMaxEncIncrS,
			m_PlatformParams.drive_param_steer.at(3).dAccIncrS2,
			m_PlatformParams.drive_param_steer.at(3).dDecIncrS2,
			m_PlatformParams.drive_param_steer.at(3).iEncOffsetIncr,
			m_PlatformParams.drive_param_steer.at(3).bIsSteer,
			m_PlatformParams.drive_param_steer.at(3).dCurrentToTorque,
			m_PlatformParams.drive_param_steer.at(3).dCurrMax,
			m_PlatformParams.drive_param_steer.at(3).iHomingDigIn);
	}

	// ------ WHEEL 1 ------ //
	// --- Motor Wheel 1 Drive
	if(m_iNumDrives >= 1)
	{
		if (m_PlatformParams.platform_config.iHasWheel1DriveMotor == 0)
		{
			// No motor
			std::cout << "node Wheel1DriveMotor available = 0" << std::endl;
		}
		else
		{
			// Motor Harmonica
			std::cout << "Wheel1DriveMotor available = type version 2" << std::endl;
			m_vpMotor[0] = new CanDriveHarmonica();
			((CanDriveHarmonica*) m_vpMotor[0])->setCanOpenParam(
				m_PlatformParams.canopen_ids.TxPDO1_W1Drive, m_PlatformParams.canopen_ids.TxPDO2_W1Drive, m_PlatformParams.canopen_ids.RxPDO2_W1Drive,
				m_PlatformParams.canopen_ids.TxSDO_W1Drive, m_PlatformParams.canopen_ids.RxSDO_W1Drive);
			m_vpMotor[0]->setCanItf(m_pCanCtrl);
			m_vpMotor[0]->setDriveParam(DriveParamW1DriveMotor);
		}
	}

	// --- Motor Wheel 1 Steer
	if(m_iNumDrives >= 1)
	{
		if (m_PlatformParams.platform_config.iHasWheel1SteerMotor == 0)
		{
			// No motor
			std::cout << "node Wheel1SteerMotor available = 0" << std::endl;
		}
		else
		{
			// Motor Harmonica
			std::cout << "Wheel1SteerMotor available = type version 2" << std::endl;
			m_vpMotor[1] = new CanDriveHarmonica();
			((CanDriveHarmonica*) m_vpMotor[1])->setCanOpenParam(
				m_PlatformParams.canopen_ids.TxPDO1_W1Steer, m_PlatformParams.canopen_ids.TxPDO2_W1Steer, m_PlatformParams.canopen_ids.RxPDO2_W1Steer,
				m_PlatformParams.canopen_ids.TxSDO_W1Steer, m_PlatformParams.canopen_ids.RxSDO_W1Steer);
			m_vpMotor[1]->setCanItf(m_pCanCtrl);
			m_vpMotor[1]->setDriveParam(DriveParamW1SteerMotor);

		}
	}
	
	// ------ WHEEL 2 ------ //
	// --- Motor Wheel 2 Drive
	if(m_iNumDrives >= 2)
	{
		if (m_PlatformParams.platform_config.iHasWheel2DriveMotor == 0)
		{
			// No motor
			std::cout << "node Wheel2DriveMotor available = 0" << std::endl;
		}
		else
		{
			// Motor Harmonica
			std::cout << "Wheel2DriveMotor available = type version 2" << std::endl;
			m_vpMotor[2] = new CanDriveHarmonica();
			((CanDriveHarmonica*) m_vpMotor[2])->setCanOpenParam(
				m_PlatformParams.canopen_ids.TxPDO1_W2Drive, m_PlatformParams.canopen_ids.TxPDO2_W2Drive, m_PlatformParams.canopen_ids.RxPDO2_W2Drive,
				m_PlatformParams.canopen_ids.TxSDO_W2Drive, m_PlatformParams.canopen_ids.RxSDO_W2Drive);
			m_vpMotor[2]->setCanItf(m_pCanCtrl);
			m_vpMotor[2]->setDriveParam(DriveParamW2DriveMotor);
		}
	}

	// --- Motor Wheel 2 Steer
	if(m_iNumDrives >= 2)
	{
		if (m_PlatformParams.platform_config.iHasWheel2SteerMotor == 0)
		{
			// No motor
			std::cout << "node Wheel2SteerMotor available = 0" << std::endl;
		}
		else
		{
			// Motor Harmonica
			std::cout << "Wheel2SteerMotor available = type version 2" << std::endl;
			m_vpMotor[3] = new CanDriveHarmonica();
			((CanDriveHarmonica*) m_vpMotor[3])->setCanOpenParam(
				m_PlatformParams.canopen_ids.TxPDO1_W2Steer, m_PlatformParams.canopen_ids.TxPDO2_W2Steer, m_PlatformParams.canopen_ids.RxPDO2_W2Steer,
				m_PlatformParams.canopen_ids.TxSDO_W2Steer, m_PlatformParams.canopen_ids.RxSDO_W2Steer);
			m_vpMotor[3]->setCanItf(m_pCanCtrl);
			m_vpMotor[3]->setDriveParam(DriveParamW2SteerMotor);

		}
	}
	
	// ------ WHEEL 3 ------ //
	// --- Motor Wheel 3 Drive
	if(m_iNumDrives >= 3)
	{
		if (m_PlatformParams.platform_config.iHasWheel3DriveMotor == 0)
		{
			// No motor
			std::cout << "node Wheel3DriveMotor available = 0" << std::endl;
		}
		else
		{
			// Motor Harmonica
			std::cout << "Wheel3DriveMotor available = type version 2" << std::endl;
			m_vpMotor[4] = new CanDriveHarmonica();
			((CanDriveHarmonica*) m_vpMotor[4])->setCanOpenParam(
				m_PlatformParams.canopen_ids.TxPDO1_W3Drive, m_PlatformParams.canopen_ids.TxPDO2_W3Drive, m_PlatformParams.canopen_ids.RxPDO2_W3Drive,
				m_PlatformParams.canopen_ids.TxSDO_W3Drive, m_PlatformParams.canopen_ids.RxSDO_W3Drive);
			m_vpMotor[4]->setCanItf(m_pCanCtrl);
			m_vpMotor[4]->setDriveParam(DriveParamW3DriveMotor);
		}
	}

	// --- Motor Wheel 3 Steer
	if(m_iNumDrives >= 3)
	{
		if (m_PlatformParams.platform_config.iHasWheel3SteerMotor == 0)
		{
			// No motor
			std::cout << "node Wheel3SteerMotor available = 0" << std::endl;
		}
		else
		{
			// Motor Harmonica
			std::cout << "Wheel3SteerMotor available = type version 2" << std::endl;
			m_vpMotor[5] = new CanDriveHarmonica();
			((CanDriveHarmonica*) m_vpMotor[5])->setCanOpenParam(
				m_PlatformParams.canopen_ids.TxPDO1_W3Steer, m_PlatformParams.canopen_ids.TxPDO2_W3Steer, m_PlatformParams.canopen_ids.RxPDO2_W3Steer,
				m_PlatformParams.canopen_ids.TxSDO_W3Steer, m_PlatformParams.canopen_ids.RxSDO_W3Steer);
			m_vpMotor[5]->setCanItf(m_pCanCtrl);
			m_vpMotor[5]->setDriveParam(DriveParamW3SteerMotor);

		}
	}
	
	// ------ WHEEL 4 ------ //
	// --- Motor Wheel 4 Drive
	if(m_iNumDrives == 4)
	{
		if (m_PlatformParams.platform_config.iHasWheel4DriveMotor == 0)
		{
			// No motor
			std::cout << "node Wheel4DriveMotor available = 0" << std::endl;
		}
		else
		{
			// Motor Harmonica
			std::cout << "Wheel4DriveMotor available = type version 2" << std::endl;
			m_vpMotor[6] = new CanDriveHarmonica();
			((CanDriveHarmonica*) m_vpMotor[6])->setCanOpenParam(
				m_PlatformParams.canopen_ids.TxPDO1_W4Drive, m_PlatformParams.canopen_ids.TxPDO2_W4Drive, m_PlatformParams.canopen_ids.RxPDO2_W4Drive,
				m_PlatformParams.canopen_ids.TxSDO_W4Drive, m_PlatformParams.canopen_ids.RxSDO_W4Drive);
			m_vpMotor[6]->setCanItf(m_pCanCtrl);
			m_vpMotor[6]->setDriveParam(DriveParamW4DriveMotor);
		}
	}

	// --- Motor Wheel 4 Steer
	if(m_iNumDrives == 4)
	{
		if (m_PlatformParams.platform_config.iHasWheel4SteerMotor == 0)
		{
			// No motor
			std::cout << "node Wheel4SteerMotor available = 0" << std::endl;
		}
		else
		{
			// Motor Harmonica
			std::cout << "Wheel4SteerMotor available = type version 2" << std::endl;
			m_vpMotor[7] = new CanDriveHarmonica();
			((CanDriveHarmonica*) m_vpMotor[7])->setCanOpenParam(
				m_PlatformParams.canopen_ids.TxPDO1_W4Steer, m_PlatformParams.canopen_ids.TxPDO2_W4Steer, m_PlatformParams.canopen_ids.RxPDO2_W4Steer,
				m_PlatformParams.canopen_ids.TxSDO_W4Steer, m_PlatformParams.canopen_ids.RxSDO_W4Steer);
			m_vpMotor[7]->setCanItf(m_pCanCtrl);
			m_vpMotor[7]->setDriveParam(DriveParamW4SteerMotor);

		}
	}
}

//-----------------------------------------------
int CanCtrlPltfCOb3::evalCanBuffer()
{
	bool bRet;
//	char cBuf[200];
	
	m_Mutex.lock();

	// as long as there is something in the can buffer -> read out next message
	while(m_pCanCtrl->receiveMsg(&m_CanMsgRec) == true)
	{
		bRet = false;
		// check for every motor if message belongs to it
		for (unsigned int i = 0; i < m_vpMotor.size(); i++)
		{
			// if message belongs to this motor write data (Pos, Vel, ...) to internal member vars
			bRet |= m_vpMotor[i]->evalReceivedMsg(m_CanMsgRec);
		}

		if (bRet == false)
		{
			std::cout << "evalCanBuffer(): Received CAN_Message with unknown identifier " << m_CanMsgRec.m_iID << std::endl;
		}		
	};
	

	m_Mutex.unlock();

	return 0;
}

//-----------------------------------------------
bool CanCtrlPltfCOb3::initPltf()
{	
	// read Configuration parameters
	readConfiguration();
		
	// Vectors for drive objects and return values
	std::vector<bool> vbRetDriveMotor;
	std::vector<bool> vbRetSteerMotor;
	std::vector<CanDriveItf*> vpDriveMotor;
	std::vector<CanDriveItf*> vpSteerMotor;
	bool bHomingOk;

	vbRetDriveMotor.assign(m_iNumDrives,0);
	vbRetSteerMotor.assign(m_iNumDrives,0);

	// Homing is done on a wheel-module base (steering and driving needs to be synchronized) 
	// copy Motor-Pointer into Steer/Drive vector for more insight
	for(int i=0; i<=m_iNumMotors; i+=2)
		vpDriveMotor.push_back(m_vpMotor[i]);
	for(int i=1; i<=m_iNumMotors; i+=2)
		vpSteerMotor.push_back(m_vpMotor[i]);

	std::vector<double> vdFactorVel;
	vdFactorVel.assign(m_iNumDrives,0);
	double dhomeVeloRadS = -1.0;


	// Start can open network
	std::cout << "StartCanOpen" << std::endl;
	sendNetStartCanOpen();
	

	// initialize drives
	
	// 1st init watchdogs
	std::cout << "Initialization of Watchdogs" << std::endl;
	for(int i=0; i<m_iNumMotors; i++)
	{
		m_vpMotor[i]->startWatchdog(true);
	}
	usleep(10000);
	
	// 2nd send watchdogs to bed while initializing drives
	for(int i=0; i<m_iNumMotors; i++)
	{
		m_vpMotor[i]->startWatchdog(false);
	}
	usleep(100000);

	std::cout << "Initialization of Watchdogs done" << std::endl;


	// ---------------------- start homing procedurs
	// Perform homing of all wheels simultaneously
	// o.k. to avoid crashing hardware -> lets check that we have at least the 8 motors, like we have on cob
	if( (int)m_vpMotor.size() == m_iNumMotors )
	{
		// Initialize and start all motors
		for (int i = 0; i<m_iNumDrives; i++)
		{
			// init + start
			vbRetDriveMotor[i] = vpDriveMotor[i]->init();
			vbRetSteerMotor[i] = vpSteerMotor[i]->init();
			usleep(10000);
			vbRetDriveMotor[i] = vbRetDriveMotor[i] && vpDriveMotor[i]->start();
			vbRetSteerMotor[i] = vbRetSteerMotor[i] && vpSteerMotor[i]->start();
			usleep(10000);
			// output State / Errors
			if (vbRetDriveMotor[i] && vbRetSteerMotor[i])
				std::cout << "Initialization of Wheel "<< (i+1) << " OK" << std::endl;
			else if (!vbRetDriveMotor[i] && vbRetSteerMotor[i])
				std::cout << "Initialization of Wheel "<< (i+1) << " ERROR while initializing DRIVE-Motor" << std::endl;
			else if (vbRetDriveMotor[i] && !vbRetSteerMotor[i])
				std::cout << "Initialization of Wheel "<< (i+1) << " ERROR while initializing STEER-Motor" << std::endl;
			else
				std::cout << "Initialization of Wheel "<< (i+1) << " ERROR while initializing STEER- and DRIVE-Motor" << std::endl;
			// Just to be sure: Set vel to zero
			vpDriveMotor[i]->setGearVelRadS(0);
			vpSteerMotor[i]->setGearVelRadS(0);
		}	
		
		// perform homing only when ALL drives are ERROR-Free
		bHomingOk = true;
		for(int i=0; i<m_iNumDrives; i++)
		{
			if((vbRetDriveMotor[i] && vbRetSteerMotor[i]) == false)
				bHomingOk = false;
		}
		if(bHomingOk)
		{
			// Calc Compensation factor for Velocity:
			if(m_iNumDrives >= 1)
				vdFactorVel[0] = - m_PlatformParams.steer_drive_coupling.at(0) + double(m_PlatformParams.platform_config.iDistSteerAxisToDriveWheelMM) / double(m_PlatformParams.platform_config.iRadiusWheelMM);
			if(m_iNumDrives >= 2)
				vdFactorVel[1] = - m_PlatformParams.steer_drive_coupling.at(1) + double(m_PlatformParams.platform_config.iDistSteerAxisToDriveWheelMM) / double(m_PlatformParams.platform_config.iRadiusWheelMM);
			if(m_iNumDrives >= 3)
				vdFactorVel[2] = - m_PlatformParams.steer_drive_coupling.at(2) + double(m_PlatformParams.platform_config.iDistSteerAxisToDriveWheelMM) / double(m_PlatformParams.platform_config.iRadiusWheelMM);
			if(m_iNumDrives == 4)
				vdFactorVel[3] = - m_PlatformParams.steer_drive_coupling.at(3) + double(m_PlatformParams.platform_config.iDistSteerAxisToDriveWheelMM) / double(m_PlatformParams.platform_config.iRadiusWheelMM);

			// initialize homing procedure
			for (int i = 0; i<m_iNumDrives; i++)
				vpSteerMotor[i]->initHoming();						
			
			// make motors move
			for (int i = 0; i<m_iNumDrives; i++)
			{
				vpSteerMotor[i]->setGearVelRadS(dhomeVeloRadS);
				vpDriveMotor[i]->setGearVelRadS(dhomeVeloRadS * vdFactorVel[i]);
			}

			// wait at least 0.5 sec.
			usleep(500000);

			// Get rid of unnecessary can messages
			bool bRet;
			do
				bRet = m_pCanCtrl->receiveMsg(&m_CanMsgRec);
			while(bRet == true);

			// arm homing procedure
			for (int i = 0; i<m_iNumDrives; i++)
				vpSteerMotor[i]->IntprtSetInt(8, 'H', 'M', 1, 1);

			// wait until all steers are homed			
			bool bAllDone, bTimeOut=false;
			int iCnt = 0;
			do
			{
				// send request for homing status
				for (int i = 0; i<m_iNumDrives; i++)
					vpSteerMotor[i]->IntprtSetInt(4, 'H', 'M', 1, 0);
				
				// eval Can Messages
				evalCanBuffer();

				// set already homed wheels to zero velocity
				bAllDone = true;
				for (int i = 0; i<m_iNumDrives; i++)
				{
					if (vpSteerMotor[i]->getStatusLimitSwitch())
					{
						vpSteerMotor[i]->setGearVelRadS(0);
						vpDriveMotor[i]->setGearVelRadS(0);
					}
					bAllDone = bAllDone && vpSteerMotor[i]->getStatusLimitSwitch();
				}	

				// increment timeout counter
				if (iCnt++ > 1000) //cpc-ck has 500 for Cob 3.5 here
					bTimeOut = true;

				// Sleep: To avoid can overload
			usleep(20000);				
			}
			while(!bAllDone && !bTimeOut);

			// Output State
			if (bTimeOut)
			{
				for (int i=0;i<m_iNumDrives;i++)
				{
					vpSteerMotor[i]->setGearVelRadS(0);
					vpDriveMotor[i]->setGearVelRadS(0);
				}
				std::cout << "Error while Homing: Timeout while waiting for homing signal" << std::endl;
			}

			// update Can Buffer once more
			// read current can buffer and update position and velocity measurements
			evalCanBuffer();

			// Now make steers move to position: zero
			// this could be handled also by the elmos themselve
			// however this way synchronization of steering and driving is better
			double m_d0 = 2.5;
			if (bTimeOut == false)
			{
				do
				{	
					bAllDone = true;
					for (int i = 0; i<m_iNumDrives; i++)
					{
						// get current position of steer
						double dCurrentPosRad;
						vpSteerMotor[i]->getGearPosRad(&dCurrentPosRad);
						// P-Ctrl					
						double dDeltaPhi = 0.0 - dCurrentPosRad; 
						// check if steer is at pos zero
						if (fabs(dDeltaPhi) < 0.03)	//alter Wert=0.03
						{
							dDeltaPhi = 0.0;
						}
						else
						{
							bAllDone = false;	
						}
						double dVelCmd = m_d0 * dDeltaPhi;
						// set Outputs
						vpSteerMotor[i]->setGearVelRadS(dVelCmd);
						vpDriveMotor[i]->setGearVelRadS(dVelCmd*vdFactorVel[i]);
					}
					usleep(20000);
					evalCanBuffer();
				} while(!bAllDone);


				// Homing done. Wheels at position zero (+/- 0.5°)
				std::cout << "Wheels homed" << std::endl;
				for (int i=0;i<m_iNumDrives;i++)
				{
					vpSteerMotor[i]->setGearVelRadS(0);
					vpDriveMotor[i]->setGearVelRadS(0);
				}
			}
		}
	}
	// ---------------------- end homing procedure

	// homing done -> wake up watchdogs
	for(int i=0; i<m_iNumMotors; i++)
	{
		m_vpMotor[i]->startWatchdog(true);
	}

	bHomingOk = true;
	for(int i=0; i<m_iNumDrives; i++)
	{
		if((vbRetDriveMotor[i] && vbRetSteerMotor[i]) == false)
		{
			bHomingOk = false;
		}
	}
	return (bHomingOk);
}

//-----------------------------------------------
bool CanCtrlPltfCOb3::resetPltf()
{
	bool bRetMotor = true;
	bool bRet = true;
	
	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		bRetMotor = m_vpMotor[i]->start();
		if (bRetMotor == true)
		{
			m_vpMotor[i]->setGearVelRadS(0);
		}
		else
		{
			std::cout << "Resetting of Motor " << i << " failed" << std::endl;
		}

		bRet &= bRetMotor;
	}
	return(bRet);
}

//-----------------------------------------------
bool CanCtrlPltfCOb3::shutdownPltf()
{

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		m_vpMotor[i]->shutdown();
	}

	return true;
}

//-----------------------------------------------
bool CanCtrlPltfCOb3::isPltfError()
{

	bool bErrMotor = false;
	std::vector<bool> vbErrMotor;
	vbErrMotor.resize(m_vpMotor.size());

	// check all motors for errors
	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		vbErrMotor[i] = m_vpMotor[i]->isError();
		// if watchdog is not yet in error state but one motor is errorneous Log Out Error-Msg
		if( (m_bWatchdogErr == false) && (vbErrMotor[i] == true) )
		{
			std::cout << "Motor " << i << " error" << std::endl;
		}
		// check whether no motor was errorneous
		bErrMotor |= vbErrMotor[i];
	}

	// if no motor is in error state force Watchdog to "No Error" state
	if(bErrMotor == false)
		m_bWatchdogErr = false;
	else
		m_bWatchdogErr = true;

	if (m_bWatchdogErr) return true;


	// Check communication
	double dWatchTime = 0;

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		dWatchTime = m_vpMotor[i]->getTimeToLastMsg();

		if(dWatchTime <  m_PlatformParams.platform_config.dCanTimeout)
		{
			m_bWatchdogErr = false;
		}
		if( (dWatchTime > m_PlatformParams.platform_config.dCanTimeout) && (m_bWatchdogErr == false) )
		{
			std::cout << "timeout CAN motor " << i << std::endl;
			m_bWatchdogErr = true;
			return true;
		}
	}

	return false;
}

//-----------------------------------------------
bool CanCtrlPltfCOb3::startWatchdog(bool bStarted)
{

	bool bRet = true;

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		bRet = m_vpMotor[i]->startWatchdog(bStarted);
	}

	return (bRet);
}

//-----------------------------------------------
void CanCtrlPltfCOb3::sendNetStartCanOpen()
{
	CanMsg msg;

	msg.m_iID  = 0;
	msg.m_iLen = 2;
	msg.set(1,0,0,0,0,0,0,0);
	m_pCanCtrl->transmitMsg(msg, false);

	usleep(100000);
}


//-----------------------------------------------
// Motor Controlers
//-----------------------------------------------


//-----------------------------------------------
int CanCtrlPltfCOb3::setVelGearRadS(int iCanIdent, double dVelGearRadS)
{		
	m_Mutex.lock();

	// If an error was detected and processed in isPltfErr().
	if (m_bWatchdogErr == true)
	{
		// Error -> Stop motor driving
		dVelGearRadS = 0;
		// The error is detected and quitted in the funtion isPltfErr().
	}

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		// check if Identifier fits to availlable hardware
		if(iCanIdent == m_viMotorID[i])
		{
			m_vpMotor[i]->setGearVelRadS(dVelGearRadS);
		}
	}
	
	m_Mutex.unlock();
	
	return 0;
}

//-----------------------------------------------
int CanCtrlPltfCOb3::requestMotPosVel(int iCanIdent)
{
	m_Mutex.lock();

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		// check if Identifier fits to availlable hardware
		if(iCanIdent == m_viMotorID[i])
		{
			m_vpMotor[i]->requestPosVel();
		}
	}
	
	m_Mutex.unlock();

	return 0;
}

//-----------------------------------------------
void CanCtrlPltfCOb3::requestDriveStatus()
{
	m_Mutex.lock();

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		m_vpMotor[i]->requestStatus();
	}

	m_Mutex.unlock();
}

//-----------------------------------------------
int CanCtrlPltfCOb3::getGearPosVelRadS(int iCanIdent, double* pdAngleGearRad, double* pdVelGearRadS)
{

	// init default outputs
	*pdAngleGearRad = 0;
	*pdVelGearRadS = 0;

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		// check if Identifier fits to availlable hardware
		if(iCanIdent == m_viMotorID[i])
		{
			m_vpMotor[i]->getGearPosVelRadS(pdAngleGearRad, pdVelGearRadS);
		}
	}
	
	return 0;
}

//-----------------------------------------------
int CanCtrlPltfCOb3::getGearDeltaPosVelRadS(int iCanIdent, double* pdAngleGearRad,
										   double* pdVelGearRadS)
{

	// init default outputs
	*pdAngleGearRad = 0;
	*pdVelGearRadS = 0;

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		// check if Identifier fits to availlable hardware
		if(iCanIdent == m_viMotorID[i])
		{
			m_vpMotor[i]->getGearDeltaPosVelRadS(pdAngleGearRad, pdVelGearRadS);
		}
	}

	return 0;
}

//-----------------------------------------------
void CanCtrlPltfCOb3::getStatus(int iCanIdent, int* piStatus, int* piTempCel)
{
	// init default outputs
	*piStatus = 0;
	*piTempCel = 0;

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		// check if Identifier fits to availlable hardware
		if(iCanIdent == m_viMotorID[i])
		{
			m_vpMotor[i]->getStatus(piStatus, piTempCel);
		}
	}

}
//-----------------------------------------------
void CanCtrlPltfCOb3::requestMotorTorque()
{
	m_Mutex.lock();

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		m_vpMotor[i]->requestMotorTorque();
	}

	m_Mutex.unlock();
}	

//-----------------------------------------------
void CanCtrlPltfCOb3::getMotorTorque(int iCanIdent, double* pdTorqueNm)
{
	// init default outputs
	*pdTorqueNm = 0;

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		// check if Identifier fits to availlable hardware
		if(iCanIdent == m_viMotorID[i])
		{
			m_vpMotor[i]->getMotorTorque(pdTorqueNm);
		}
	}

}
//-----------------------------------------------
void CanCtrlPltfCOb3::setMotorTorque(int iCanIdent, double dTorqueNm)
{
	m_Mutex.lock();

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		// check if Identifier fits to availlable hardware
		if(iCanIdent == m_viMotorID[i])
		{
			m_vpMotor[i]->setMotorTorque(dTorqueNm);
		}
	}

	m_Mutex.unlock();
}

//-----------------------------------------------
// Read out Elmo-Recorder via CAN (cpc-pk)
//-----------------------------------------------

//-----------------------------------------------
int CanCtrlPltfCOb3::ElmoRecordings(int iFlag, int iParam, std::string sString) {
	int tempRet = 0;
	int bRet = 0;
	
	switch(iFlag) {
		case 0: //Flag = 0 means reset recorder and configure it
			for(unsigned int i = 0; i < m_vpMotor.size(); i++) {
				m_vpMotor[i]->setRecorder(0, iParam); //Configure Elmo Recorder with RecordingGap and start immediately
			}
			return 0;

		case 1: //Flag = 1 means start readout process, mustn't be called too early (while Rec is in process..)
			for(unsigned int i = 0; i < m_vpMotor.size(); i++) {
				if((tempRet = m_vpMotor[i]->setRecorder(1, iParam, sString)) > bRet) { 
					bRet = tempRet; //Query Readout of Index to Log Directory
				}
			}
			return bRet;
		
		case 99:
			for(unsigned int i = 0; i < m_vpMotor.size(); i++) {
				m_vpMotor[i]->setRecorder(99, 0); //Stop any ongoing SDO transfer and clear corresponding data.
			}
			return 0;
			
		case 100:
			for(unsigned int i = 0; i < m_vpMotor.size(); i++) {
				bRet += m_vpMotor[i]->setRecorder(2, 0); //Request state of transmission
			}
			return bRet;
		
		default:
			return -1;
	}
}
