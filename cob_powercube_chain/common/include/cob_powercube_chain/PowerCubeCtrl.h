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
 * ROS package name: cob_powercube_chain
 * Description: An interface class to the Powercube-hardware implementing armInterface.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Felix Geibel, email:Felix.Geibel@gmx.de
 * Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 *
 * Date of creation: Apr 2007
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

#ifndef __POWER_CUBE_CTRL_H_
#define __POWER_CUBE_CTRL_H_

//#define __LINUX__

#include <libm5api/m5apiw32.h>
#include <cob_powercube_chain/moveCommand.h>
#include <cob_powercube_chain/PowerCubeCtrlParams.h>
//#include "Utilities/mutex.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>

//#include <pthread.h>

// using namespace std;
// Needs the Following Libraries to Compile:
// -lm5api
// -ldevice
// -lutil
// -lntcan


//-------------------------------------------------------------------------
//                              Defines
// -------------------------------------------------------------------------
class PowerCubeCtrlParams
{

	public:
		PowerCubeCtrlParams():m_DOF(0){;}
		
		int Init(std::string CanModule, std::string CanDevice, int BaudRate, std::vector<int> ModuleIDs)
		{
			SetCanModule(CanModule);
			SetCanDevice(CanDevice);
			SetBaudRate(BaudRate);	
			SetNumberOfDOF(ModuleIDs.size());
			for (unsigned int i=0; i < GetNumberOfDOF() ;i++)
			{
				m_IDModulesNumber.push_back(ModuleIDs[i]);
			}
			return 0;
		}
				
		//DOF	
		void SetNumberOfDOF(unsigned int DOF){m_DOF=DOF;}
		unsigned int GetNumberOfDOF(){return m_DOF;}

		//Can Module
		void SetCanModule(std::string CanModule){m_CanModule = CanModule;}
		std::string GetCanModule(){return m_CanModule;}

		//Can Device
		void SetCanDevice(std::string CanDevice){m_CanDevice = CanDevice;}
		std::string GetCanDevice(){return m_CanDevice;}
			
		//BaudRate
		void SetBaudRate(int BaudRate){m_BaudRate=BaudRate;}
		int GetBaudRate(){return m_BaudRate;}
	
		//ModuleIDs
		std::vector<int> GetModuleIDVector(){return m_IDModulesNumber;}
		int GetModuleID(unsigned int no){if (no < GetNumberOfDOF()) return m_IDModulesNumber[no]; else return -1;}
		int SetModuleID(unsigned int no, int id){
			if (no < GetNumberOfDOF()) 
			{
				m_IDModulesNumber[no] = id;
				return 0; 
			}
			else 
				return -1;

		}
		
		//Angular Constraints
		int SetUpperLimits(std::vector<double> UpperLimits)
		{
			if (UpperLimits.size() == GetNumberOfDOF())
			{
				m_UpperLimits = UpperLimits;
				return 0;

			}
			return -1;
		}
		int SetLowerLimits(std::vector<double> LowerLimits)
		{
			if (LowerLimits.size() == GetNumberOfDOF())
			{
				m_LowerLimits = LowerLimits;
				return 0;
			}
			return -1;
		}
		int SetAngleOffsets(std::vector<double> AngleOffsets)
		{
			if (AngleOffsets.size() == GetNumberOfDOF())
			{
				m_Offsets = AngleOffsets;
				return 0;
			}
			return -1;
		}
		int SetMaxVel(std::vector<double> MaxVel)
		{
			if (MaxVel.size() == GetNumberOfDOF())
			{
				m_MaxVel = MaxVel;
				return 0;
			}
			return -1;
		}
		int SetMaxAcc(std::vector<double> MaxAcc)
		{
			if (MaxAcc.size() == GetNumberOfDOF())
			{
				m_MaxAcc = MaxAcc;
				return 0;
			}
			return -1;
		}
		
		std::vector<double> GetUpperLimits(){return m_UpperLimits;}
		std::vector<double> GetLowerLimits(){return m_LowerLimits;}
		std::vector<double> GetAngleOffsets(){return m_Offsets;}
		std::vector<double> GetMaxAcc(){return m_MaxAcc;}
		std::vector<double> GetMaxVel(){return m_MaxVel;}
		
		
	private:
		std::vector<int> m_IDModulesNumber;
		unsigned int m_DOF;
		std::string m_CanModule;
		std::string m_CanDevice;
		int m_BaudRate;
		std::vector<double> m_Offsets;
		std::vector<double> m_UpperLimits;
		std::vector<double> m_LowerLimits;
		std::vector<double> m_MaxVel;
		std::vector<double> m_MaxAcc;
};

/* uncomment the following line to switch on debugging output: */
// #define _POWER_CUBE_CTRL_DEBUG

class PowerCubeCtrl
{
	public:
		
		PowerCubeCtrl();
		~PowerCubeCtrl();
		
		bool Init(PowerCubeCtrlParams * params);

		bool isInitialized() const { return m_Initialized; }

		std::string getErrorMessage() const { return m_ErrorMessage; }

		bool Close();

		/////////////////////////////////
		// Funktionen Arm-Ansteuerung: //
		/////////////////////////////////
		
		/// @brief same as MoveJointSpace, but final angles should by reached simultaniously!
		/// Returns the time that the movement will take
		bool MoveJointSpaceSync(const std::vector<double>& Angle);
		
		/// @brief Moves all cubes by the given velocities
		bool MoveVel(const std::vector<double>& Vel);
		
		/// @brief Stops the Manipulator immediately
		bool Stop();
		
		/// @brief Recovery after emergency stop or power supply failure
		bool Recover();
		
		///////////////////////////////////////////
		// Funktionen zum setzen von Parametern: //
		///////////////////////////////////////////
		
		/// @brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
		/// A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
		bool setMaxVelocity(double radpersec);
		bool setMaxVelocity(const std::vector<double>& radpersec);
		
		/// @brief Sets the maximum angular acceleration (rad/s^2) for the Joints, use with care!
		/// A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
		bool setMaxAcceleration(double radPerSecSquared);
		bool setMaxAcceleration(const std::vector<double>& radPerSecSquared);
		
		////////////////////////////////////////////
		// hier die Funktionen zur Statusabfrage: //
		////////////////////////////////////////////
	

		/// @brief Returns the current Joint Angles
		bool getConfig(std::vector<double>& result);
		
		/// @brief Returns the current Angular velocities (Rad/s)
		bool getJointVelocities(std::vector<double> & result);
		
		/// @brief Returns true if any of the Joints are still moving
		/// Should also return true if Joints are accelerating or decelerating
		bool statusMoving();
		
		/// @brief Returns true if any of the Joints are decelerating
		bool statusDec();
		
		/// @brief Returs true if any of the Joints are accelerating
		bool statusAcc();

		/// @brief Waits until all Modules are homed, writes status comments to out.
		bool doHoming();
		bool HomingDone();
		
		typedef enum
		{
			PC_CTRL_OK = 0,
			PC_CTRL_NOT_REFERENCED = -1,
			PC_CTRL_ERR = -2,
			PC_CTRL_POW_VOLT_ERR = -3
		} PC_CTRL_STATE;
        
	    bool getStatus(PC_CTRL_STATE& error, std::vector<std::string>& errorMessages);
        
		/// @brief Tells the Modules not to start moving until PCubel_startMotionAll is called
		bool waitForSync();
		/// @brief Execute move commands immediately from now on:
		bool dontWaitForSync();

	protected:
		

		/// @brief Returns the time for a ramp-move about dtheta with v, a would take, assuming the module is
		/// currently moving at vnow.
		void millisleep(unsigned int milliseconds) const;

		int m_DOF;
		int m_Dev;
		bool m_Initialized;
		bool m_CANDeviceOpened;
		
		std::vector<int> m_IdModules;
		
		std::vector<double> m_maxVel;
		std::vector<double> m_maxAcc;

		std::string m_ErrorMessage;
		
};


#endif
