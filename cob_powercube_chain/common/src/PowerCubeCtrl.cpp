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
 * ROS package name: powercube_chain
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

#include <powercube_chain/PowerCubeCtrl.h>
#include <string>
#include <sstream>
#include <time.h>
#include <cmath>
#ifdef PYTHON_THREAD
#include <Python.h>
#endif
//#define __LINUX__

#define DEG 57.295779524
#define MANUAL_AXES0_OFFSET  1.8
#define MANUAL_AXES6_OFFSET 1.5

#define PCTRL_CHECK_INITIALIZED() \
if ( isInitialized()==false )													\
{																		\
    m_ErrorMessage.assign("Manipulator not initialized.");              \
	return false;														\
}

using namespace std;

/* initialize static class member: */
//mutex PowerCubeCtrl::ms_PowercubeIOMutex;

PowerCubeCtrl::PowerCubeCtrl()
{
	m_Dev=0;
	m_DOF = 1;
	m_CANDeviceOpened = false;
	m_Initialized = false;
}


bool PowerCubeCtrl::Init(PowerCubeCtrlParams * params)
{
	int CanDevice= 0;
	int CanBaudRate = 0;
	std::vector<double> offsets;
	std::vector<double> upperLimits;
	std::vector<double> lowerLimits;
	if (params != NULL)
	{
		m_DOF=params->GetNumberOfDOF();
		m_IdModules = params->GetModuleIDVector();
		CanBaudRate = params->GetBaudRate();
		CanDevice = params->GetCanDevice();
		m_maxAcc = params->GetMaxAcc();
		upperLimits = params->GetUpperLimits();
		lowerLimits = params->GetLowerLimits();
		m_maxVel = params->GetMaxVel();
		offsets = params->GetAngleOffsets();
/*		for (int i = 0; i< m_DOF; i++)
		{
			upperLimits.push_back(3.1);
			lowerLimits.push_back(-3.1);
			offsets.push_back(0);
			m_maxVel.push_back(0.8);
		}
*/		
		
	}
	else
	{
   		std::cout <<"PowerCubeCtrl::Init: Error, parameters == NULL"<<endl; 
		return false;
	}
	std::cout<<"=========================================================================== "<<endl;
	std::cout<<"PowerCubeCtrl:Init: Successfully initialized with the following parameters: "<<endl;
	std::cout<<"DOF: "<<m_DOF<<endl;
	std::cout<<"CanBaudRate: "<<CanBaudRate<<endl;
	std::cout<<"CanDevice: "<<CanDevice<<endl;
	std::cout<<"Ids: ";
	for (int i = 0; i< m_DOF; i++)
	{
			std::cout<<m_IdModules[i]<<" ";
	}
	std::cout<<endl<<"maxVel: ";
	for (int i = 0; i< m_DOF; i++)
	{
			std::cout<<m_maxVel[i]<<" ";
	}
	std::cout<<endl<<"maxAcc: ";
	for (int i = 0; i< m_DOF; i++)
	{
			std::cout<<m_maxAcc[i]<<" ";
	}
	std::cout<<endl<<"upperLimits: ";
	for (int i = 0; i< m_DOF; i++)
	{
			std::cout<<upperLimits[i]<<" ";
	}
	std::cout<<endl<<"lowerLimits: ";
	for (int i = 0; i< m_DOF; i++)
	{
			std::cout<<lowerLimits[i]<<" ";
	}
	std::cout<<endl<<"offsets: ";
	for (int i = 0; i< m_DOF; i++)
	{
			std::cout<<offsets[i]<<" ";
	}
	std::cout<<endl<<"=========================================================================== "<<endl;
	ostringstream initStr;
	initStr << "PCAN:" << CanDevice << "," << CanBaudRate;
	std::cout << "initstring = " << initStr.str().c_str() << std::endl;
	int ret = 0;
	ret = PCube_openDevice (&m_Dev, initStr.str().c_str());
	
	if (ret != 0)
	{
		ostringstream errorMsg;
		errorMsg << "Could not open device, m5api error code: " << ret;
		m_ErrorMessage = errorMsg.str();
		return false;
	}
	m_CANDeviceOpened = true;
	
    PCube_resetAll(m_Dev);
	
	// Make sure m_IdModules is clear of Elements:
	m_IdModules.clear();
	
	for(int i=0; i<m_DOF; i++)
	{

		// Check if the Module is connected:
		unsigned long serNo;

		int ret = PCube_getModuleSerialNo( m_Dev, m_IdModules[i], &serNo );
		
		// if not return false ( ret == 0 means success)
		if( ret != 0 )
		{
			ostringstream errorMsg;
			errorMsg << "Could not find Module with ID " << m_IdModules[i];
			m_ErrorMessage = errorMsg.str();
			return false;
		}
		
		// otherwise success, save Id in m_IdModules
		cout << "found module " << m_IdModules[i] << endl;
	}
	
	vector<string> errorMessages;
	PC_CTRL_STATE status;
	getStatus(status, errorMessages);
	if ((status != PC_CTRL_OK) && (status != PC_CTRL_NOT_REFERENCED))
	{
		m_ErrorMessage.assign("");
		for (int i=0; i<m_DOF; i++)
		{
			m_ErrorMessage.append(errorMessages[i]);
			m_ErrorMessage.append("\n");
		}
		return false;
	}
	else if (status == PC_CTRL_NOT_REFERENCED) 
	{
		std::cout << "PowerCubeCtrl:Init: Homing is executed ...\n";
		bool successful = false;
		successful = doHoming();
		if (!successful)
		{
			std::cout << "PowerCubeCtrl:Init: homing not successful, aborting ...\n";
			return false;
		}
	}


	// Set angle offsets
	for (int i = 0; i < m_DOF; i++)
	{
		PCube_setHomeOffset(m_Dev,m_IdModules[i],offsets[i]);
	}
	
	// Set Limits in hardware
	for (int i = 0; i < m_DOF; i++)
	{
		PCube_setMinPos(m_Dev, m_IdModules[i], lowerLimits[i]);
		PCube_setMaxPos(m_Dev, m_IdModules[i], upperLimits[i]);
	}
	
	/* Default Werte für maximalgechw. & Beschl. setzen: */
	setMaxVelocity(m_maxVel);
	setMaxAcceleration(m_maxAcc);
	
	// Bewegungen sollen Synchron gestartet werden:
	waitForSync();
	
	m_Initialized = true;
	
	return true;
}

/** The Deconstructor 
 */
PowerCubeCtrl::~PowerCubeCtrl()
{ 
	if (m_CANDeviceOpened)
	{
		PCube_closeDevice(m_Dev);
	}
}

/// @brief Returns the current Joint Angles
bool PowerCubeCtrl::getConfig(std::vector<double>& result)
{
    PCTRL_CHECK_INITIALIZED();
    
    result.resize(m_DOF);

	PCube_savePosAll(m_Dev);	
	
	for (int i=0; i < m_DOF; i++)
	{
		float pos;
		PCube_getSavePos(m_Dev, m_IdModules[i], &pos);
		result[i]=pos;
	}
	
	return true;
}

/// @brief Returns the current Angular velocities (Rad/s)
bool PowerCubeCtrl::getJointVelocities(std::vector<double>& result)
{
    PCTRL_CHECK_INITIALIZED();
    
	result.resize(m_DOF);

	for (int i=0; i < m_DOF; i++)
	{
		float pos;
		PCube_getVel(m_Dev, m_IdModules[i], &pos);
		result[i] = pos;
	}
	
	return true;
}

bool PowerCubeCtrl::MoveJointSpaceSync(const std::vector<double>& target)
{
    PCTRL_CHECK_INITIALIZED();
	
	vector<string> errorMessages;
	PC_CTRL_STATE status;
	getStatus(status, errorMessages);
	if ((status != PC_CTRL_OK))
	{
		m_ErrorMessage.assign("");
		for (int i=0; i<m_DOF; i++)
		{
			m_ErrorMessage.append(errorMessages[i]);
			m_ErrorMessage.append("\n");
		}
		return false;
	}

	// Evtl. Fragen zur Rechnung / zum Verfahren an: Felix.Geibel@gmx.de
	std::vector<double> acc(m_DOF);
	std::vector<double> vel(m_DOF);
	
	double TG = 0;
	
	try 
	{
		
		// Ermittle Joint, der bei max Geschw. und Beschl. am längsten braucht:
		int DOF = m_DOF;
		
		std::vector<double> posnow;
		if ( getConfig(posnow) == false )
		    return false;
		
		    
		std::vector<double> velnow;
		if ( getJointVelocities(velnow) == false )
		    return false;
			
		std::vector<double> times(DOF);

		for (int i=0; i < DOF; i++)
		{
			RampCommand rm(posnow[i], velnow[i], target[i], m_maxAcc[i], m_maxVel[i]);
			times[i] = rm.getTotalTime();
		}
		
		// determine the joint index that has the greates value for time
		int furthest = 0;
		
		double max = times[0];
	
	    for (int i=1; i<m_DOF; i++)
	    {
		    if (times[i] > max)
		    {
			    max = times[i];
			    furthest = i;
		    }
	    }

		RampCommand rm_furthest(posnow[furthest], velnow[furthest], target[furthest], m_maxAcc[furthest], m_maxVel[furthest]);
		
		double T1 = rm_furthest.T1();
		double T2 = rm_furthest.T2();
		double T3 = rm_furthest.T3();
	
		// Gesamtzeit:
		TG = T1 + T2 + T3;
		
		// Jetzt Geschwindigkeiten und Beschl. für alle: 
		acc[furthest] = m_maxAcc[furthest];
		vel[furthest] = m_maxVel[furthest];
		
		for (int i = 0; i < DOF; i++)
		{
			if (i != furthest)
			{
				double a; double v;
				// a und v berechnen:
				RampCommand::calculateAV(
					posnow[i], 
					velnow[i], 
					target[i], 
					TG, T3, 
					m_maxAcc[i],
					m_maxVel[i],
					a, 
					v);
							
				acc[i] = a;
				vel[i] = v;
			}
		}
	} 
	catch(...) 
	{
		return false;
	}
	
	// Send motion commands to hardware	
	for (int i=0; i < m_DOF; i++)
	{
		PCube_moveRamp(m_Dev, m_IdModules[i], target[i], fabs(vel[i]), fabs(acc[i]));
	}
	
	PCube_startMotionAll(m_Dev);

	return true;	
}
		
bool PowerCubeCtrl::MoveVel(const std::vector<double>& vel)
{
    PCTRL_CHECK_INITIALIZED();
	
	vector<string> errorMessages;
	PC_CTRL_STATE status;
	getStatus(status, errorMessages);
	if ((status != PC_CTRL_OK))
	{
		m_ErrorMessage.assign("");
		for (int i=0; i<m_DOF; i++)
		{
			m_ErrorMessage.append(errorMessages[i]);
			m_ErrorMessage.append("\n");
		}
		return false;
	}

	for (int i=0; i < m_DOF; i++)
	{
		PCube_moveVel(m_Dev, m_IdModules[i], vel[i] );
	}
	PCube_startMotionAll(m_Dev);
	
	return true;
}
		
				
/// @brief Stops the Manipulator immediately
bool PowerCubeCtrl::Stop()
{
    // stop should be executes without checking any conditions
	PCube_haltAll(m_Dev);
	
	// Nach halt nehmen die Modules keine Bewegungsbefehle mehr an 
	// muessen also resettet werden, damit Bewegung wieder möglich. 
	
	millisleep(50);
	
	PCube_resetAll(m_Dev);
	
	return true;
}

/// @brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
/// A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
bool PowerCubeCtrl::setMaxVelocity(double radpersec) 
{ 	
    PCTRL_CHECK_INITIALIZED();
	for (int i=0; i<m_DOF; i++)
	{
		//m_maxVel[i] = radpersec;
		PCube_setMaxVel(m_Dev, m_IdModules[i], radpersec);
	}
	
	return true;
}

bool PowerCubeCtrl::setMaxVelocity(const std::vector<double>& radpersec) 
{ 
    PCTRL_CHECK_INITIALIZED();
    
	for (int i=0; i<m_DOF; i++)
	{
		//m_maxVel[i] = radpersec[i];
		PCube_setMaxVel(m_Dev, m_IdModules[i], radpersec[i]);
	}
	
	return true;
}

/// @brief Sets the maximum angular acceleration (rad/s^2) for the Joints, use with care!
/// A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
bool PowerCubeCtrl::setMaxAcceleration(double radPerSecSquared)
{
    PCTRL_CHECK_INITIALIZED();
	
	for (int i=0; i<m_DOF; i++)
	{
		m_maxAcc[i] = radPerSecSquared;
		PCube_setMaxAcc(m_Dev, m_IdModules[i], radPerSecSquared);
	}

    return true;
}
		
bool PowerCubeCtrl::setMaxAcceleration(const std::vector<double>& radPerSecSquared)
{ 
    PCTRL_CHECK_INITIALIZED();
    	
	for (int i=0; i<m_DOF; i++)
	{
		m_maxAcc[i] = radPerSecSquared[i];
		PCube_setMaxAcc(m_Dev, m_IdModules[i], radPerSecSquared[i]);
	}

    return true;
}
		


/// @brief Returns true if some cubes are still moving
bool PowerCubeCtrl::statusMoving()
{
    PCTRL_CHECK_INITIALIZED();
    
	for(int i=0; i<m_DOF; i++)
	{	
		unsigned long status;
		
		PCube_getModuleState(m_Dev,m_IdModules[i], &status);
		
		if (status & STATEID_MOD_MOTION)
			return true;
	}
	return false;
}
		
/// @brief Returns true if any of the Joints are decelerating
bool PowerCubeCtrl::statusDec()
{
    PCTRL_CHECK_INITIALIZED();
    
	for (int i=0; i<m_DOF; i++)
	{
		unsigned long status;
		
		PCube_getModuleState(m_Dev,m_IdModules[i], &status);
		
		if (status & STATEID_MOD_RAMP_DEC)
			return true;
	}
	return false;
}


/// @brief Returs true if any of the Joints are accelerating
bool PowerCubeCtrl::statusAcc()
{
    PCTRL_CHECK_INITIALIZED();
    
	for (int i=0; i<m_DOF; i++)
	{
		unsigned long status;
		
		PCube_getModuleState(m_Dev,m_IdModules[i], &status);
		
		if (status & STATEID_MOD_RAMP_ACC)
			return true;
	}
	return false;
}

/// @brief does homing for all Modules
bool PowerCubeCtrl::doHoming()
{
    //PCTRL_CHECK_INITIALIZED();

	for(int i=0; i<m_DOF; i++)
	{
		cout << "Module " << m_IdModules[i] << " homed" << endl;
		PCube_homeModule(m_Dev, m_IdModules[i]);
	}
	 
	return true;
}
		
/** Wait functions which is waiting that all cubes have done their homing 
 */
bool PowerCubeCtrl::HomingDone()
{
    PCTRL_CHECK_INITIALIZED();
    
	for(int i=0; i<m_DOF; i++)
	{
		unsigned long int help;
		do
		{
			PCube_getModuleState(m_Dev, m_IdModules[i], &help);
			
			millisleep(100);
		} while ( (help & STATEID_MOD_HOME) == 0 );
	}
	
	return true;
}

bool PowerCubeCtrl::waitForSync()
{
    if (m_CANDeviceOpened)
    {
	    for (int i=0; i < m_DOF; i++)
	    {
		    unsigned long confword;
		
		    PCube_getConfig(m_Dev, m_IdModules[i], &confword );
		    PCube_setConfig(m_Dev, m_IdModules[i], confword | CONFIGID_MOD_SYNC_MOTION);
	    }
	    return true;
	}
	else
	{
	    return false;
	}
}

bool PowerCubeCtrl::dontWaitForSync()
{
    if (m_CANDeviceOpened)
    {
	    for (int i=0; i < m_DOF; i++)
	    {
		    unsigned long confword;
		
		    PCube_getConfig(m_Dev, m_IdModules[i], &confword );
		    PCube_setConfig(m_Dev, m_IdModules[i], confword & (~CONFIGID_MOD_SYNC_MOTION));
	    }
	    return true;
	}
	else
	{
	    return false;
	}
}

bool PowerCubeCtrl::getStatus(PC_CTRL_STATE& error, vector<string>& errorMessages)
{
	errorMessages.clear();
	errorMessages.resize(m_DOF);
        
    error = PC_CTRL_OK;

	for(int i=0; i<m_DOF; i++)
	{
		unsigned long int state;
		
		PCube_getModuleState(m_Dev, m_IdModules[i], &state);
                
		if (state & STATEID_MOD_POW_VOLT_ERR)
		{
			ostringstream errorMsg;
			errorMsg << "Error in Module " << m_IdModules[i] << ": ";
			errorMsg << "Motor voltage below minimum value!";
			errorMessages[i] = errorMsg.str();
			error = PC_CTRL_POW_VOLT_ERR;
		}
		else if (!(state & STATEID_MOD_HOME))
        {
			ostringstream errorMsg;
			errorMsg << "Warning: Module " << m_IdModules[i];
			errorMsg << " is not referenced!";
			errorMessages[i] = errorMsg.str();
            error = PC_CTRL_NOT_REFERENCED;
        }
 		else if (state & STATEID_MOD_ERROR)
		{
			ostringstream errorMsg;
			errorMsg << "Error in  Module " << m_IdModules[i];
			errorMsg << " : Status code: " << hex << state;
			errorMessages[i] = errorMsg.str();
 			error = PC_CTRL_ERR;
		}
		else
		{
			ostringstream errorMsg;
			errorMsg << "Module with Id " << m_IdModules[i];
			errorMsg << ": Status OK.";
			errorMessages[i] = errorMsg.str();
		}
	}
    return true;
}

bool PowerCubeCtrl::Close()
{
    if (m_CANDeviceOpened)
    {
	    m_Initialized = false;
	    m_CANDeviceOpened = false;
	
        PCube_closeDevice(m_Dev);
	
	    return true;
	}
	else
	{
	    return false;
	}
}

void PowerCubeCtrl::millisleep(unsigned int milliseconds) const
{
	timespec warten;
	// Millisekunden in Sekunden und Nanosekunden aufteilen
	warten.tv_sec = milliseconds / 1000;
	warten.tv_nsec = (milliseconds % 1000) * 1000000;
	timespec gewartet;
	nanosleep(&warten, &gewartet);
}
