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
 * Description: This class simulates the PowerCubes in a very rough and simple way.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Felix Geibel, email:Felix.Geibel@gmx.de
 * Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 *
 * Date of creation: Aug 2007
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

#ifndef __POWER_CUBE_SIM_H_
#define __POWER_CUBE_SIM_H_

#include <powercube_chain/PowerCubeCtrl.h>
#include <powercube_chain/Joint.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

using namespace std;
#include <pthread.h>


//-------------------------------------------------------------------------
//                              Defines
// -------------------------------------------------------------------------

#define MAX_VEL 0.5
#define MAX_ACC 0.5
#define K13 0.5
#define K14 0.5
#define K15 3
#define DAEMPFUNG 0.5

//#define __LINUX__

#ifdef SWIG
%module PowerCubeSim
%include "Source/Manipulation/Interfaces/armInterface.h"
%{
	#include "PowerCubeSim.h"
%}
#endif 

class PowerCubeSim;

/* Thread arguments for simulation threads*/
typedef struct
{
	PowerCubeSim * cubeSimPtr;
	int cubeID;
	double targetAngle;
} SimThreadArgs;

class PowerCubeSim
{
	public:
		
		PowerCubeSim();
		~PowerCubeSim();
		

		bool Init(PowerCubeCtrlParams * params);

		bool isInitialized() const { return m_Initialized; }

		std::string getErrorMessage() const { return m_ErrorMessage; }

		int Close(){return true;}
		// Arm-Ansteuerung:
		

		/// @brief same as MoveJointSpace, but final angles should by reached simultaniously!
		/// Returns the time that the movement will take
		bool MoveJointSpaceSync(const std::vector<double>& Angle);
				
		/// @brief Moves all cubes by the given velocities
		bool MoveVel(const std::vector<double>& vel);
		
		/// @brief Stops the Manipulator immediately
		bool Stop();
		
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
		bool getJointVelocities(std::vector<double>& result);

		void setCurrentAngles(std::vector<double> Angles);

		void setCurrentJointVelocities(std::vector<double> Angles);
		
		/// @brief Returns true if any of the Joints are still moving
		/// Should also return true if Joints are accelerating or decelerating
		bool statusMoving();
		bool statusMoving(int cubeNo);
		
		/// @brief Returns true if any of the Joints are decelerating
		bool statusDec();
		
		/// @brief Returs true if any of the Joints are accelerating
		bool statusAcc();

		/// @brief Looks for connected Modules and returns their Ids in a vector
		//vector<int> getModuleMap(int dev);
		
		/// @brief Waits until all Modules are homed, writes status comments to out.
		//void HomingDone();
		
                typedef enum
                {
                        PC_CTRL_OK = 0,
                        PC_CTRL_NOT_REFERENCED = -1,
                        PC_CTRL_ERR = -2,
			PC_CTRL_POW_VOLT_ERR = -3
                } PC_CTRL_STATE;
                //int getStatus(){return PC_CTRL_OK;}
		double maxVel;

		void setStatusMoving (int cubeNo, bool moving);
		bool getStatusMoving (int cubeNo) const { return m_MovementInProgress[cubeNo]; }

		vector<int>  getModuleMap() const {return m_IdModules;}
	        std::vector<double>  getCurrentAngularMaxVel() {return m_CurrentAngularMaxVel;}
	        std::vector<double>  getCurrentAngularMaxAccel() {return m_CurrentAngularMaxAccel;}
	    void millisleep(unsigned int milliseconds) const;
	protected:
		
		/// @brief Tells the Modules not to start moving until PCubel_startMotionAll is called
		//void waitForSync();
		/// @brief Execute move commands immediately from now on:
		//void dontWaitForSync();
		/// @brief Returns the time for a ramp-move about dtheta with v, a would take, assuming the module is currently moving at vnowClose 
		double timeRampMove(double dtheta, double vnow, double v, double a);

		int startSimulatedMovement(std::vector<double> target);


		//void* SimThreadRoutine (void*); 	
	
	
		#ifdef COB3
		Manipulator * m_Obj_Manipulator;
#endif
		
		int m_DOF;
		int m_Initialized;
		int m_NumOfModules;
		int m_Dev;
		vector<int> m_IdModules;
		
		std::vector<double> m_maxVel;
		std::vector<double> m_maxAcc;

		Jointd m_AngleOffsets;

		std::vector<bool> m_MovementInProgress;

		std::vector<double> m_CurrentAngles;
		std::vector<double> m_CurrentAngularVel;
		std::vector<double> m_CurrentAngularMaxVel;
		std::vector<double> m_CurrentAngularMaxAccel;



		//vector<unsigned long> startConf;
		
		std::string m_ErrorMessage;

		float maxAcc;

		pthread_mutex_t  m_Angles_Mutex;
		pthread_mutex_t  m_AngularVel_Mutex;
		pthread_mutex_t  m_Movement_Mutex;
		pthread_t  * m_SimThreadID;
		SimThreadArgs ** m_SimThreadArgs;
		
};


#endif
