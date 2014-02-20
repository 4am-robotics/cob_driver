/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2010 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: schunk_modular_robotics
 * \note
 *   ROS stack name: schunk_modular_robotics
 * \note
 *   ROS package name: schunk_powercube_chain
 *
 * \author
 *   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * \author
 *   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 *
 * \date Date of creation: Dec 2010
 *
 * \brief
 *   Implementation of powercube control.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef __POWER_CUBE_CTRL_H_
#define __POWER_CUBE_CTRL_H_

// standard includes
#include <iostream>
#include <sstream>
#include <string>
#include <deque>

// own includes
#include <cob_lookat_chain/LookatParams.h>

class LookatCtrl
{

public:

	/// Constructor
	LookatCtrl(LookatParams * params)
	{
		m_params = params;
		m_Initialized = false;
	}

	/// Destructor
	~LookatCtrl();

	bool Init(LookatParams * params)
	{
		m_params = params;
		
		unsigned int DOF = m_params->GetDOF();
		m_status.resize(DOF);
		m_positions.resize(DOF);
		m_velocities.resize(DOF);
		m_accelerations.resize(DOF);
		
		m_Initialized = true;
	}

	////////////////////////////
	// Functions for control: //
	////////////////////////////
	/*!
	 * \brief Moves all cubes by the given velocities
	 */
	bool MoveVel(const std::vector<double>& velocities)
	{
		float delta_t = ros::Time::now().toSec() - m_last_time_pub.toSec();
		m_last_time_pub = ros::Time::now();  
		
		for(unsigned int i=0; i<m_params->GetDOF(); i++)
		{
			m_positions[i] += m_velocities[i] * delta_t;
		}
		//ROS_INFO("New velocities:");
		//for(unsigned int i=0; i<velocities.size(); i++)
			//ROS_INFO("\t%d: %f", i, velocities[i]);
		m_velocities = velocities;
		return true;
	}

	//////////////////////////////////
	// functions to set parameters: //
	//////////////////////////////////

	/*!
	 * \brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
	 *
	 * A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
	 */
	bool setMaxVelocity(double velocity);
	bool setMaxVelocity(const std::vector<double>& velocities);

	/*!
	 * \brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
	 *
	 * A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
	 */
	bool setMaxAcceleration(double acceleration);
	bool setMaxAcceleration(const std::vector<double>& accelerations);

	/////////////////////////////////////////////////
	// Functions for getting state and monitoring: //
	/////////////////////////////////////////////////

	/*!
	 * \brief Returns the state of all modules
	 */
	bool updateStates()
	{
		float delta_t = ros::Time::now().toSec() - m_last_time_pub.toSec();
		m_last_time_pub = ros::Time::now();  
		
		for(unsigned int i=0; i<m_params->GetDOF(); i++)
		{
			m_positions[i] += m_velocities[i] * delta_t;
		}
		
		return true;
	}

	/*!
	 * \brief Returns true if any of the Joints are still moving
	 *
	 * Should also return true if Joints are accelerating or decelerating
	 */
	bool statusMoving()
	{
		for(unsigned int i=0; i<m_params->GetDOF(); i++)
		{
			if(m_velocities[i] != 0)
				return true;
		}
		
		return false;
	}

	/*!
	 * \brief Gets the current positions
	 */
	std::vector<double> getPositions()
	{
		return m_positions;
	}

	/*!
	 * \brief Gets the current velcities
	 */
	std::vector<double> getVelocities()
	{
		//ROS_INFO("Current velocities:");
		//for(unsigned int i=0; i<m_velocities.size(); i++)
			//ROS_INFO("\t%d: %f", i, m_velocities[i]);
		return m_velocities;
	}

	/*!
	 * \brief Gets the current accelerations
	 */
	std::vector<double> getAccelerations()
	{
		return m_accelerations;
	}

protected:
	bool m_Initialized;

	LookatParams* m_params;

	std::vector<unsigned long> m_status;
	std::vector<double> m_positions;
	//std::deque< std::vector<double> > m_cached_pos;
	std::vector<double> m_velocities;
	std::vector<double> m_accelerations;

	ros::Time m_last_time_pub;
};

#endif
