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
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 * Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 *
 * Date of creation: Dec 2009
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

#include "PowerCubeSim_OROCOS.h"
#include "simulatedArm.h"
#include <vector>

using namespace RTT;

PowerCubeSim_OROCOS::PowerCubeSim_OROCOS(std::string name) : OrocosRTTArmDriverInterface(name)
{

}

PowerCubeSim_OROCOS::~PowerCubeSim_OROCOS()
{
}

bool PowerCubeSim_OROCOS::configureHook()
{
	if ( m_powercubectrl.Init(false) )
	{
		log(Info) << "PowerCubeSim initialized successfully." << endlog();
		return true;
	}
	else
	{
		log(Info) << "Error while initializing PowerCubeSim:" << endlog();
		//log(Info) << m_powercubectrl.getErrorMessage() <<  endlog();
		return false;
	}
}

bool PowerCubeSim_OROCOS::startHook()
{
 /*   if ( m_in_Angles.connected() )
    {
    	// only one of the input ports should be used simultaniously
    	if ( m_in_Velocities.connected() || m_in_Currents.connected() ) {
    		log(Info) << "Error in PowerCubeSim.startHook(): more than one input port is connected!" << endlog();
    		return false;
    	}
    	m_in_Angles_connected = true;
    	log(Info) << "PowerCubeSim succesfully detected connection at in_Angles" << endlog();
	    return true;
    }
    if ( m_in_Velocities.connected() )
    {
    	// only one of the input ports should be used simultaniously
    	if ( m_in_Angles.connected() || m_in_Currents.connected() ) {
    		log(Info) << "Error in PowerCubeSim.startHook(): more than one input port is connected!" << endlog();
    		return false;
    	}
    	m_in_Velocities_connected = true;
    	log(Info) << "PowerCubeSim succesfully detected connection at in_Velocities" << endlog();
	    return true;
    }
    if ( m_in_Currents.connected() )
    {
    	log(Info) << "Error, port \"setCur_R\" is connected to PowerCubeSim_OROCOS.\"";
    	log(Info) << "Current movement is not yet supported by PowerCubeSim." << endlog();
    	return false;
    	/*
    	// only one of the input ports should be used simultaniously
    	if ( m_in_Angles.connected() || m_in_Velocities.connected() ) {
    		log(Info) << "Error in PowerCubeSim.startHook(): more than one input port is connected!" << endlog();    		
    		return false;
    	}
    	m_in_Current_connected = true;
	    return true;

    }
    else
    {
    	log(Info) << "No Input port is connected, PowerCubeSim will only return the current state." << endlog();
    }    */
    return true;
}

void PowerCubeSim_OROCOS::updateHook()
{
	//log(Info) << "updateHook is being executed." << endlog();
	Jointd setpositions, setvelocities;
	setpositions = set_position_inport.Get();
	setvelocities = set_velocity_inport.Get();
	if(setpositions.size() == 7)
	{
	
	}
	else if(setvelocities.size() == 7)
	{

	}
	
	current_position_outport.Set(m_powercubectrl.getConfig());
	current_velocity_outport.Set(m_powercubectrl.getJointVelocities());
}

void PowerCubeSim_OROCOS::stopHook()
{
	stopArm();
}

bool PowerCubeSim_OROCOS::stopArm()
{
    //stop
    m_powercubectrl.stop();
    return true;
}

bool PowerCubeSim_OROCOS::isArmStopped()
{
    //isStopped
    if(m_powercubectrl.statusMoving())
        return false;
    return true;
}
