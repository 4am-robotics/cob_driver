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
 * Description: An interface class to a virtual manipulator.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Felix Geibel, email:Felix.Geibel@gmx.de
 * Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 *
 * Date of creation: Oct 2007
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

#include <powercube_chain/simulatedArm.h>
#include <powercube_chain/simulatedMotor.h>
#include <math.h>

#define PSIM_CHECK_INITIALIZED() \
if ( isInitialized()==false )											\
{																		\
    m_ErrorMessage.assign("Manipulator not initialized.");              \
	return false;														\
}

simulatedArm::simulatedArm()
{
	std::cerr << "==============Starting Simulated Arm\n";
	m_DOF = 0;
	m_Initialized = false;
	m_motors.clear();
			
}

simulatedArm::~simulatedArm()
{
	// nothing to do...
	;
}

bool simulatedArm::Init(PowerCubeCtrlParams * params)
{
	m_DOF = 7;
	
	double vmax = 1.0;
	double amax = 1.5;
	
	setMaxVelocity(vmax);
	setMaxAcceleration(amax);
	m_maxVel.resize(m_DOF);
	m_maxAcc.resize(m_DOF);
	
	std::vector<double> ul(m_DOF);
	std::vector<double> ll(m_DOF);
	
	for (int i=0; i < m_DOF; i++)
	{
		ul[i] = 115;
		ll[i] = -ul[i];
		m_maxVel[i] = vmax;
		m_maxAcc[i] = amax;
	}
	
	for (int i=0; i < m_DOF; i++)
	{
		ul[i] *= M_PI/180.0;
	}
	
	for (int i=0; i < m_DOF; i++)
	{
		m_motors.push_back( simulatedMotor(ll[i], ul[i], amax, vmax) );
	}
	std::cerr << "===========Initializing Simulated Powercubes\n";
	m_Initialized = true;
	return true;
}

/// @brief Stops the Manipulator immediately
bool simulatedArm::Stop()
{
	for (int i=0; i < m_DOF; i++)
		m_motors[i].stop();
	return true;
}

bool simulatedArm::MoveJointSpaceSync(const std::vector<double>& target)
{
	std::cerr << "======================TUTUTUTUT\n";
    PSIM_CHECK_INITIALIZED();


	std::vector<double> acc(m_DOF);
	std::vector<double> vel(m_DOF);
	
	double TG = 0;
	

	try 
	{		
		// Ermittle Joint, der bei max Geschw. und Beschl. am längsten braucht:

		std::vector<double> posnow;
		posnow.resize(m_DOF);
		if ( getConfig(posnow) == false )
		    return false;
		    
		std::vector<double> velnow;
		velnow.resize(m_DOF);
		if ( getJointVelocities(velnow) == false )
		    return false;
			
		std::vector<double> times(m_DOF);

		for (int i=0; i < m_DOF; i++)
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
		
		for (int i = 0; i < m_DOF; i++)
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
		m_ErrorMessage.assign("Problem during calculation of a and av.");
		return false;
	}
		
	// Jetzt Bewegung starten:	
	for (int i=0; i < m_DOF; i++)
	{
		std::cout << "moving motor " << i << ": " << target[i] << ": " << vel[i] << ": " << acc[i] << "\n";
		m_motors[i].moveRamp(target[i], vel[i], acc[i]);
	}
	
	return true;	
}

bool simulatedArm::MoveVel(const std::vector<double>& Vel)
{
    PSIM_CHECK_INITIALIZED();

	for (int i=0; i < m_DOF; i++)
		if(Vel[i] > 0.0)
			m_motors[i].moveVel(Vel[i]);
	
	return true;
}	


bool simulatedArm::MovePos(const std::vector<double>& Pos)
{
    PSIM_CHECK_INITIALIZED();

	for (int i=0; i < m_DOF; i++)
		m_motors[i].movePos(Pos[i]);
	
	return true;
}	

	
///////////////////////////////////////////
// Funktionen zum setzen von Parametern: //
///////////////////////////////////////////

/// @brief Sets the maximum angular velocity (rad/s) for all Joints, use with care!
bool simulatedArm::setMaxVelocity(double radpersec)
{
    PSIM_CHECK_INITIALIZED();
    
    m_maxVel.resize(m_DOF);

	for (int i=0; i < m_DOF; i++)
	{
		m_maxAcc[i] = radpersec;
		m_motors[i].setMaxVelocity(radpersec);
	}
	
	return true;
}

bool simulatedArm::setMaxVelocity(const std::vector<double>& radpersec)
{
    PSIM_CHECK_INITIALIZED();
    
    m_maxAcc = radpersec;

	for (int i=0; i < m_DOF; i++)
		m_motors[i].setMaxVelocity(radpersec[i]);
	
	return true;
}

/// @brief Sets the maximum angular acceleration (rad/s^2) all the Joints, use with care!
bool simulatedArm::setMaxAcceleration(double radPerSecSquared)
{
    PSIM_CHECK_INITIALIZED();

	for (int i=0; i < m_DOF; i++)
		m_motors[i].setMaxAcceleration(radPerSecSquared);
	
	return true;
}

bool simulatedArm::setMaxAcceleration(const std::vector<double>& radPerSecSquared)
{
    PSIM_CHECK_INITIALIZED();

	for (int i=0; i < m_DOF; i++)
		m_motors[i].setMaxAcceleration(radPerSecSquared[i]);
	
	return true;
}


////////////////////////////////////////////
// hier die Funktionen zur Statusabfrage: //
////////////////////////////////////////////
		
		
/// @brief Returns the current Joint Angles
bool simulatedArm::getConfig(std::vector<double>& result)
{
    PSIM_CHECK_INITIALIZED();

	result.resize(m_DOF);
	for (int i=0; i < m_DOF; i++)
	{
		result[i] = m_motors[i].getAngle();
	}
	
	return true;
}	

/// @brief Returns the current Angular velocities (Rad/s)
bool simulatedArm::getJointVelocities(std::vector<double>& result)
{
    PSIM_CHECK_INITIALIZED();

	result.resize(m_DOF);
	for (int i=0; i < m_DOF; i++)
	{
		result[i] = m_motors[i].getVelocity();
	}
	
	return true;
}	

/// @brief Returns true if any of the Joints are still moving
/// Should also return true if Joints are accelerating or decelerating
bool simulatedArm::statusMoving()
{
    PSIM_CHECK_INITIALIZED();

	for(int i=0; i<m_DOF; i++)
	{	
		if (m_motors[i].statusMoving())
			return true;
	}
	return false;
}
		

/// @brief Returns true if any of the Joints are decelerating
bool simulatedArm::statusDec()
{
    PSIM_CHECK_INITIALIZED();

	for(int i=0; i<m_DOF; i++)
	{	
		if (m_motors[i].statusDec())
			return true;
	}
	return false;
}

/// @brief Returs true if any of the Joints are accelerating
bool simulatedArm::statusAcc()
{
    PSIM_CHECK_INITIALIZED();

	for(int i=0; i<m_DOF; i++)
	{	
		if (m_motors[i].statusAcc())
			return true;
	}
	return false;
}

