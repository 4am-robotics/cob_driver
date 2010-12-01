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



#ifndef __ELMO_CTRL_H__
#define __ELMO_CTRL_H__
#define NTCAN_CLEAN_NAMESPACE
#include <cob_canopen_motor/CanDriveItf.h>
#include <cob_canopen_motor/CanDriveHarmonica.h>
#include <cob_generic_can/CanItf.h>
#include <cob_generic_can/CanPeakSys.h>
#include <cob_generic_can/CanPeakSysUSB.h>
#include <cob_generic_can/CanESD.h>

// Headers provided by cob-packages which should be avoided/removed^M
#include <cob_utilities/IniFile.h>
#include <string>


typedef struct _CanOpenAddress
{
	int TxPDO1;
	int TxPDO2;
	int RxPDO2;
	int TxSDO;
	int RxSDO;
} CanOpenAddress;

class ElmoCtrl;

typedef struct
{
	ElmoCtrl * pElmoCtrl;
} ElmoThreadArgs;


class ElmoCtrlParams
{

	public:
		ElmoCtrlParams(){;}
		
		int Init(std::string CanDevice, int BaudRate, int ModuleID)
		{
			SetCanDevice(CanDevice);
			SetBaudRate(BaudRate);	
			SetModuleID(ModuleID);
			return 0;
		}
				
		//Can Device
		void SetCanDevice(std::string CanDevice){m_CanDevice = CanDevice;}
		std::string GetCanDevice(){return m_CanDevice;}
			
		//BaudRate
		void SetBaudRate(int BaudRate){m_BaudRate=BaudRate;}
		int GetBaudRate(){return m_BaudRate;}
	
		//ModuleIDs
		int GetModuleID(){return m_ModuleID;}
		void SetModuleID(int id){m_ModuleID = id;}
		
		//Angular Constraints
		void SetUpperLimit(double UpperLimit){m_UpperLimit = UpperLimit;}
		void SetLowerLimit(double LowerLimit){m_LowerLimit = LowerLimit;}
		void SetAngleOffset(double AngleOffset){m_Offset = AngleOffset;}
		double GetUpperLimit(){return m_UpperLimit;}
		double GetLowerLimit(){return m_LowerLimit;}
		double GetAngleOffset(){return m_Offset;}

		//Velocity
		void SetMaxVel(double maxVel){m_MaxVel = maxVel;}
		double GetMaxVel(){return m_MaxVel;}

		//HomingDir
		void SetHomingDir(int dir){m_HomingDir = dir;}
		int GetHomingDir(){return m_HomingDir;}

		//HomingDigIn
		void SetHomingDigIn(int dig_in){m_HomingDigIn = dig_in;}
		int GetHomingDigIn(){return m_HomingDigIn;}

		//CanIniFile
		void SetCanIniFile(std::string iniFile){m_CanIniFile=iniFile;}
		std::string GetCanIniFile(){return m_CanIniFile;}
		
		
	private:
		int m_ModuleID;
		std::string  m_CanDevice;
		int m_BaudRate;
		int m_HomingDir;
		int m_HomingDigIn;
		double m_Offset;
		double m_UpperLimit;
		double m_LowerLimit;
		double m_MaxVel;
		std::string m_CanIniFile;

};

class ElmoCtrl
{

public:
	ElmoCtrl();
	~ElmoCtrl();

	bool Init(ElmoCtrlParams* params, bool home = true);


	double MoveJointSpace (double PosRad);
	

	bool Home();

	bool RecoverAfterEmergencyStop();

	bool Stop();


	void setMaxVelocity(float radpersec)
	{
		m_MaxVel = radpersec;
	}

	/**
	 * Gets the position and velocity.
	 * @param pdAngleGearRad joint-position in radian
	 * @param pdVelGearRadS joint-velocity in radian per second
	 */
	int getGearPosVelRadS(double* pdAngleGearRad, double* pdVelGearRadS);
	
	/**
	 * Sets required position and veolocity.
	 * Use this function only in position mode.
	 */
	int setGearPosVelRadS(double dPosRad, double dVelRadS);

	/**
	 * Triggers evaluation of the can-buffer.
	 */
	int evalCanBuffer();

	//_double getConfig();

	double getJointVelocity();



	bool SetMotionCtrlType(int type);
	int GetMotionCtrlType(); 
	

	enum {
		POS_CTRL,
		VEL_CTRL
	} MOTION_CTRL_TYPE;


	CANPeakSysUSB* GetCanCtrl(){return m_CanCtrl;}
	//CanESD* GetCanCtrl(){return m_CanCtrl;}
	//bool m_ElmoCtrlThreadActive;
	/// @brief joint mutexes
	//pthread_mutex_t   m_cs_elmoCtrlIO;
	CanDriveHarmonica * m_Joint;  

private:

	bool sendNetStartCanOpen(CanItf* canCtrl);

	int m_MotionCtrlType;	

	DriveParam * m_JointParams;

	int  m_CanBaseAddress;
	CanOpenAddress  m_CanAddress;

	CANPeakSysUSB * m_CanCtrl;
	//CanESD * m_CanCtrl;

	double m_MaxVel;
	int m_HomingDir;
	int m_HomingDigIn;

	double  m_UpperLimit;
	double  m_LowerLimit;
	double  m_JointOffset;
	/*	
	Jointd* m_CurrentAngularVelocity;
	/// @brief current joint angles
	Jointd* m_CurrentJointAngles;
	*/
	/// @brief joint mutexes
	pthread_mutex_t   m_Mutex;
	/// @brief joint velocity mutexes
	//pthread_mutex_t   m_AngularVel_Mutex;
	/// @brief  Thread IDs for PowerCube connection threads
	//pthread_t   m_ElmoThreadID;
	/// @brief Arguments for Powercube connection Threads
	//ElmoThreadArgs * m_ElmoCtrlThreadArgs;
	ElmoCtrlParams * m_Params;
	CanMsg m_CanMsgRec;
	
	
	/// @brief the logfile for debugging info & errors:

};









#endif //__ELMO_CTRL_H__
