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

#include "PowerCubeCtrl_OROCOS.h"

using namespace RTT;

PowerCubeCtrl_OROCOS::PowerCubeCtrl_OROCOS(std::string name, std::string xmlFile, int dof)
: TaskContext(name),
	m_xmlFile(xmlFile),
	m_dof(dof),
	// initialize dataports:
	m_in_Angles("in_Angles"),
	m_in_Velocities("in_Velocities"),
	m_in_Currents("in_Currents"),
	m_out_Angles("out_Angles"),  // note: initial value
	m_out_Velocities("out_Velocities"),  // note: initial value
	// initialize method
	m_stop_method("stop_movement",&PowerCubeCtrl_OROCOS::stopArm, this),
	//m_moveJointSpace_method("moveJointSpace", &PowerCubeCtrl_OROCOS::moveJointSpace, this),
	m_CanDev_prop("CanDev_n", "Can Device No.", 0),
	m_CanBaud_prop("BaudRate", "Can Baud Rate", 1000),
	m_dof_prop("DOF", "Degrees of Freedom", 7),
	m_MaxVel_prop("Max_Vel", "Maximum Velocity", 0.5),
	m_MaxAcc_prop("Max_Acc", "Maximum Acceleration", 0.8),
	m_powercubectrl()
{
    this->ports()->addPort( &m_in_Angles, "Port for desired Angles" );
    m_in_Angles_connected = false;
    this->ports()->addPort( &m_in_Velocities, "Port for desired velocities" );
    m_in_Velocities_connected = false;
    this->ports()->addPort( &m_in_Currents, "Port for desired currents" );
    m_in_Currents_connected = false;

    this->ports()->addPort( &m_out_Angles, "Port for current positions" );
    this->ports()->addPort( &m_out_Velocities, "Port for current velocities" );

    this->methods()->addMethod(&m_stop_method, "Stop the Arm");
    //this->methods()->addMethod(&m_moveJointSpace_method, "execute synchronized ramp commands");
    
	this->properties()->addProperty(&m_CanDev_prop);
	this->properties()->addProperty(&m_CanBaud_prop);
	this->properties()->addProperty(&m_dof_prop);
	this->properties()->addProperty(&m_MaxVel_prop);
	this->properties()->addProperty(&m_MaxAcc_prop);

	//m_mod_params.clear();

	m_modId_props.clear();
  	m_ul_props.clear();
  	m_ll_props.clear();
	m_offset_props.clear();
	
	for (int i=0; i<m_dof; i++)
	{
		ostringstream os;

		os << "modId" << i+1;
		m_modId_props.push_back( Property<int>( os.str(), os.str(), 0 ) );

		os.str("");	os.clear();
		os << "upperLimit" << i+1;
		m_ul_props.push_back( Property<double>( os.str(), os.str(), 0.0 ) );

		os.str("");	os.clear();
		os << "lowerLimit" << i+1;
		m_ll_props.push_back( Property<double>( os.str(), os.str(), 0.0 ) );
		
		os.str("");	os.clear();
		os << "offset" << i+1;
		m_offset_props.push_back( Property<double>( os.str(), os.str(), 0.0 ) );

	}

	m_mod_params.clear();
	for (int i=0; i<m_dof; i++)
	{
		ostringstream os;
		os << "modParams" << i+1;

		PropertyBag bag( os.str() );
		bag.add( &m_modId_props[i] );
		bag.add( &m_ul_props[i] );
		bag.add( &m_ll_props[i] );
		bag.add( &m_offset_props[i] );

		Property<PropertyBag>  prop_of_bag( os.str(), "", bag);

		m_mod_params.push_back( prop_of_bag );
	}

	for (int i=0; i<m_dof; i++)
	{
		this->properties()->addProperty( &(m_mod_params[i]) );
	}
    //m_stopped = false;
}

PowerCubeCtrl_OROCOS::~PowerCubeCtrl_OROCOS()
{
	stop();
}

bool PowerCubeCtrl_OROCOS::configureHook()
{
	this->marshalling()->readProperties(m_xmlFile);

	// COPY PROPERTIES INTO STRUCT POWERCUBEPARAMETERS	

	PowerCubeParameters pcparams;
	
	pcparams.modIds.resize(m_dof);
	pcparams.upperlimits.resize(m_dof);
	pcparams.lowerlimits.resize(m_dof);
	pcparams.offsets.resize(m_dof);
	
	pcparams.dof=m_dof_prop.get();	
	pcparams.CanDevice=m_CanDev_prop.get();
	pcparams.BaudRate=m_CanBaud_prop.get();
	pcparams.maxVel=m_MaxVel_prop.get();
	pcparams.maxAcc=m_MaxAcc_prop.get(); 
	
	int i;
	if(m_dof != pcparams.dof)
	{
		log(Info) << "Degrees of freedom" << m_dof << "doesn't match with xml input DOF." << pcparams.dof << endlog();
	    return false;
	}
	
	for (i=0 ; i< (m_dof) ; i++)
	{
		pcparams.modIds[i] = m_modId_props[i].get();
		pcparams.upperlimits[i] = m_ul_props[i].get();
		pcparams.lowerlimits[i] = m_ll_props[i].get();
		pcparams.offsets[i] = m_offset_props[i].get();
	}

//	int id = m_modId_props[3].get();
	
	if ( m_powercubectrl.Init(pcparams) )
	{
		log(Info) << "PowerCubeCtrl initialized successfully." << endlog();
		return true;
	}
	else
	{
		log(Info) << "Error while initializing PowerCubeCtrl:" << endlog();
		log(Info) << m_powercubectrl.getErrorMessage() <<  endlog();
		return false;
	}
	return true;
}

void PowerCubeCtrl_OROCOS::stopHook()
{
	//m_stopped = true;
	stopArm();
}

bool PowerCubeCtrl_OROCOS::startHook()
{
    if ( m_in_Angles.connected() )
    {
    	// only one of the input ports should be used simultaniously
    	if ( m_in_Velocities.connected() || m_in_Currents.connected() ) {
    		log(Info) << "Error in PowerCubeSim.startHook(): more than one input port is connected!" << endlog();
    		return false;
    	}
    	m_in_Angles_connected = true;
    	log(Info) << "PowerCubeSim succesfully detected connection at in_Angles" << endlog();
    	//m_stopped = false;
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
    	//m_stopped = false;
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
    	//m_stopped = false;
	    return true;
	    */
    }
    else
    {
    	log(Info) << "No Input port is connected, PowerCubeSim will only return the current state." << endlog();
    }
    return true;
}

void PowerCubeCtrl_OROCOS::updateHook()
{
	// log(Info) << "updateHook is being executed." << endlog();
	
	// Execute desired movements:
	if ( m_in_Angles_connected )
	{
		std::vector<double> angles_desired(7);
		angles_desired = m_in_Angles.Get();
	    m_powercubectrl.MovePos( angles_desired );
	}
	
	if ( m_in_Velocities_connected )
	{
		std::vector<double> vel_desired(7);
		vel_desired = m_in_Velocities.Get();
	    m_powercubectrl.MoveVel( vel_desired );
	}
	if ( m_in_Currents_connected )
	{
		std::vector<double> currents_desired(7);
		currents_desired = m_in_Currents.Get();
	    m_powercubectrl.MoveCur( currents_desired );
	}

    // get current angles & velocities
    std::vector<double> curConfig(7);
	m_powercubectrl.getConfig(curConfig);
    m_out_Angles.Set(curConfig);

    std::vector<double> curVelocities(7);
    m_powercubectrl.getJointVelocities(curVelocities);
    m_out_Velocities.Set(curVelocities);	
}

bool PowerCubeCtrl_OROCOS::stopArm()
{
    //stop
    m_powercubectrl.Stop();		
    return true;
}

bool PowerCubeCtrl_OROCOS::moveJointSpace(std::vector<double> target)		
{
	return m_powercubectrl.MoveJointSpaceSync(target);
}

bool PowerCubeCtrl_OROCOS::isArmStopped()
{
	return !m_powercubectrl.statusMoving();
}
