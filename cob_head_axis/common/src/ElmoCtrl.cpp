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
 * ROS stack name: cob3_driver
 * ROS package name: cob_camera_axis
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Ulrich Reiser, email:ulrich.reiser@ipa.fhg.de
 *
 * Date of creation: Jul 2010
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


#include <cob_camera_axis/ElmoCtrl.h>

#include <iostream>

using namespace std;

void Sleep(int msecs){usleep(1000*msecs);}

bool ElmoCtrl::sendNetStartCanOpen(CanItf* canCtrl) {
	bool ret = false;	

	CanMsg msg;

	msg.m_iID  = 0;
	msg.m_iLen = 2;
	msg.set(1,0,0,0,0,0,0,0);
	ret = canCtrl->transmitMsg(msg, false);

	usleep(100000);

	return ret;
}


ElmoCtrl::ElmoCtrl() {
	m_Joint = NULL;
	m_JointParams = NULL;
	m_CanCtrl = NULL;
	m_CanBaseAddress = NULL;
	m_MotionCtrlType = POS_CTRL;
	m_MaxVel = 2.0;
	m_Params = NULL;
}

ElmoCtrl::~ElmoCtrl() {
	if (m_Joint)
		m_Joint->shutdown();
		delete m_Joint;
	if (m_JointParams)
		delete m_JointParams;
	if (m_CanCtrl)
		delete m_CanCtrl;
}

bool ElmoCtrl::Home()
{
	bool success = false;
	if (m_Joint != NULL) {
		m_Joint->initHoming();
	}

	//ToDo: UHR: necessary?
	Sleep(10);
	int HomingDir = m_Params->GetHomingDir();
	printf("ElmoCtrl: Home(): Homing Vel = %f\n",HomingDir*0.3);
	m_Joint->setGearVelRadS(HomingDir*0.3);
	//ToDo: UHR: necessary?
	Sleep(750);
	success = m_Joint->execHoming();
	m_Joint->setGearVelRadS(0.0);

	return success;
}


int ElmoCtrl::evalCanBuffer() {
	bool bRet;
	
	//pthread_mutex_lock(&(m_Mutex));

	// as long as there is something in the can buffer -> read out next message
	while(m_CanCtrl->receiveMsg(&m_CanMsgRec) == true) {
		bRet = false;
		// check if the message belongs to camera_axis motor
		bRet |= m_Joint->evalReceivedMsg(m_CanMsgRec);

		if (bRet == true) {
		} else std::cout << "cob_camera_axis: Unknown CAN-msg: " << m_CanMsgRec.m_iID << "  " << (int)m_CanMsgRec.getAt(0) << " " << (int)m_CanMsgRec.getAt(1) << std::endl;
	}
	
	//pthread_mutex_unlock(&(m_Mutex));

	return 0;
}

bool ElmoCtrl::RecoverAfterEmergencyStop() {
	
	bool success = false;
	printf("Resetting motor ...\n");
	success = m_Joint->start();
	if (!success) {
		printf("failed!\n");
	} else {	
		printf("successful\n");
		m_Joint->setGearVelRadS(0);
	}
	Sleep(1000);
	return success;
}


bool ElmoCtrl::Init(ElmoCtrlParams * params, bool home) { //home = true by default
	bool success = false;
	string CanIniFile;
	string CanDevice;
	int baudrate = 0;
	
	m_Params = params;

	if (params == NULL) {
		printf("ElmoCtrlParams:Error:%s:%d:, invalid parameters!\n",__FILE__,__LINE__);
		success = false;
	} else {
		success = true;
	}

	if (success) {
		printf( "------------ ElmoCtrl Init ---------------\n");
		
		//Allocate memory and read params e.g. gotten from ROS parameter server
		m_Joint = new CanDriveHarmonica();
		m_JointParams = new DriveParam();
		m_CanBaseAddress = params->GetModuleID();
		CanIniFile = params->GetCanIniFile();	
		m_MaxVel = params->GetMaxVel();
		m_HomingDir = params->GetHomingDir();
		
		if (CanIniFile.length() == 0) {	
			printf("%s,%d:Error: Parameter 'CanIniFile' not given!\n",__FILE__,__LINE__);
			success = false;
		}
		
		CanDevice = params->GetCanDevice();
		if (CanDevice.length() == 0) {	
			printf("%s,%d:Error: Parameter 'Can-Device' not given!\n",__FILE__,__LINE__);
			success = false;
		}
		
		baudrate = params->GetBaudRate();
		if (baudrate == 0) {	
			printf("%s,%d:Error: Parameter 'Baud-Rate' not given!\n",__FILE__,__LINE__);
			success = false;
		}
		
		//Setting motor model data		
		if (success) {
			m_JointOffset = params->GetAngleOffset();
			m_UpperLimit = params->GetUpperLimit();
			m_LowerLimit = params->GetLowerLimit();
		}

		if (success) {
			printf("The following parameters were successfully read from the parameter server (given through *params): \n");
			printf("CanIniFile: 	%s\n",CanIniFile.c_str());
			printf("CanDieODvice: 	%s\n",CanDevice.c_str());
			printf("Baudrate: 	%d\n",baudrate);
			printf("Module ID: %d\n",m_CanBaseAddress);
			printf("Max Vel: %f\n",m_MaxVel);
			printf("Homing Dir: %d\n",m_HomingDir);
			printf("Offset/Limit(min/max)  %f/(%f,%f)\n",m_JointOffset,m_LowerLimit,m_UpperLimit);
		}
	}
	
	//Setting up CAN interface
	
	if (success) {
	  //m_CanCtrl = new CanESD(CanIniFile.c_str(), false);
		m_CanCtrl = new CANPeakSysUSB(CanIniFile.c_str());
		if (m_CanCtrl == NULL) {
			printf("%s,%d:Error: Could not open Can Device!\n",__FILE__,__LINE__);
			success = false;
		}
	  }
	
	if (success) {
			/* WRONG CAN-identifiers
			//m_CanBaseAddress = params->GetModulID(i);
			m_CanAddress.TxPDO1 = 0x181 + m_CanBaseAddress -1;
			m_CanAddress.TxPDO2 = 0x285 + m_CanBaseAddress -1;
			m_CanAddress.RxPDO2 = 0x301 + m_CanBaseAddress -1;
			m_CanAddress.TxSDO = 0x491 + m_CanBaseAddress -1;
			m_CanAddress.RxSDO = 0x511 + m_CanBaseAddress -1;
			*/
			m_CanAddress.TxPDO1 = 0x181 + m_CanBaseAddress -1;
			m_CanAddress.TxPDO2 = 0x281 + m_CanBaseAddress -1;
			m_CanAddress.RxPDO2 = 0x301 + m_CanBaseAddress -1;
			m_CanAddress.TxSDO = 0x581 + m_CanBaseAddress -1;
			m_CanAddress.RxSDO = 0x601 + m_CanBaseAddress -1;
			m_Joint->setCanItf(m_CanCtrl);
			m_Joint->setCanOpenParam(m_CanAddress.TxPDO1, 
							m_CanAddress.TxPDO2, 
							m_CanAddress.RxPDO2, 
							m_CanAddress.TxSDO, 
							m_CanAddress.RxSDO );
							
			printf("CanAdresses set to %d (Base), %x, %x, %x, %x, %x...\n", m_CanBaseAddress,
																		m_CanAddress.TxPDO1,
																		m_CanAddress.TxPDO2,
																		m_CanAddress.RxPDO2,
																		m_CanAddress.TxSDO,
																		m_CanAddress.RxSDO);
		
	  }
	  
	  if (success) {
	  	success = sendNetStartCanOpen(m_CanCtrl);
	  }
	  
	  if (success) {
		//ToDo: Read from File!
		/*
		int iDriveIdent,
		int iEncIncrPerRevMot,
		double dVelMeasFrqHz,
		double dBeltRatio,
		double dGearRatio,
		int iSign,
		double dVelMaxEncIncrS,
		double dAccIncrS2,
		double dDecIncrS2,
		int iEncOffsetIncr,
		bool bIsSteer,
        double dCurrToTorque,
		double dCurrMax,
		int iHomingDigIn
		*/
		m_JointParams->setParam( //parameters taken from CanCtrl.ini
							0, //int iDriveIdent,
							4096,//int iEncIncrPerRevMot,
							1,//double dVelMeasFrqHz,
							1,//double dBeltRatio,
							47.77,//double dGearRatio,
							-1.0,//int iSign,
							740000,//double dVelMaxEncIncrS,
							1000000,//80000,//double dAccIncrS2,
							1000000,//80000//double dDecIncrS2),
					  		0, //int iEncOffsetIncr
					  		true, // bool bIsSteer
					  		0, // double dCurrToTorque
					  		0, // double dCurrMax
					  		5 // int iHomingDigIn
					  );
					  
		m_Joint->setDriveParam(*m_JointParams);
		}

	  if (success)
	  {
			  printf("Init motor ...\n");
			  success = m_Joint->init();
			  if (!success)
			  {
					  printf("failed!\n");
			  }
			  else
			  {	
					  printf("successful\n");
			  }
	  }
	  if (success)
			  success = SetMotionCtrlType(m_MotionCtrlType);
			  if(!success) std::cout << "Failed to SetMotionCtrlType to " << m_MotionCtrlType << std::endl;

	  if (success)
	  {
			  printf("Starting motor ..\n");
			  success = m_Joint->start();
			  if (!success)
			  {
					  printf("failed!\n");
			  }
			  else
			  {	
					  printf("successful\n");
					  m_Joint->setGearVelRadS(0);
			  }
	  }
	  //ToDo: UHR: necessary?
	  Sleep(1000);

	  if (success && home)
	  {
	  	std::cout << "Start homing procedure now.." << std::endl;
			  success = Home();
	  }
	  //Thread init
	  if (success)
	  {
			  pthread_mutex_init(&m_Mutex,NULL);
	  }


	  return success;
}

bool ElmoCtrl::SetMotionCtrlType(int type)
{
	m_MotionCtrlType = type;
	bool success = false; 
	if (type == POS_CTRL)
	{
			success = m_Joint->shutdown();
			if (success)
					success = m_Joint->setTypeMotion(CanDriveHarmonica::MOTIONTYPE_POSCTRL);
			//ToDo: necessary?
			Sleep(100);
			success = m_Joint->start();

	}
	else if (type == VEL_CTRL)
	{
		//UHR: ToDo
		printf("%s%d:Error: Velocity control not implemented yet!\n",__FILE__,__LINE__);
		success = false;
	}
	return success;
};


int ElmoCtrl::GetMotionCtrlType() 
{
	return m_MotionCtrlType;
}

int ElmoCtrl::getGearPosVelRadS( double* pdAngleGearRad, double* pdVelGearRadS)
{

	// init default outputs
	*pdAngleGearRad = 0;
	*pdVelGearRadS = 0;

	m_Joint->getGearPosVelRadS(pdAngleGearRad, pdVelGearRadS);
	//*pdAngleGearRad += m_JointOffset;
	
	return 0;
}

//-----------------------------------------------

int ElmoCtrl:: setGearPosVelRadS(double dPosRad, double dVelRadS)
{
	if(dPosRad< m_LowerLimit) {
		std::cout << "Position under LowerBound -> set up" << std::endl;
		dPosRad = m_LowerLimit;
	} else if(dPosRad > m_UpperLimit) {
		std::cout << "Position over UpperBound -> set down" << std::endl;
		dPosRad = m_UpperLimit;
	}
		
	if(dVelRadS > m_MaxVel)
		dVelRadS = m_MaxVel;
	else if(dVelRadS < -m_MaxVel)
		dVelRadS = -m_MaxVel;

	//m_Joint->setGearPosVelRadS(dPosRad + m_JointOffset, dVelRadS);
	
	m_Joint->setGearPosVelRadS(-3.141592654 - dPosRad + Offset_, dVelRadS);

	return 0;
}

bool ElmoCtrl::Stop()
{
		//UHR: ToDo: what happens exactly in this method? Sudden stop?
		double pos = 0.0;
		double vel = 0.0;
		m_Joint->getGearPosVelRadS(&pos,&vel);
		m_Joint->setGearPosVelRadS(pos,0);

		return true;
		//m_Joint[i]->shutdown();

}
