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

#ifndef OROCOSRTTARMDRIVERINTERFACE_H_
#define OROCOSRTTARMDRIVERINTERFACE_H_
#include "Joint.h"
#include <rtt/TaskContext.hpp>
#include <rtt/Command.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Method.hpp>

using namespace RTT;

class OrocosRTTArmDriverInterface : public TaskContext
{

public:



	ReadDataPort<Jointd>  set_position_inport;
	ReadDataPort<Jointd>  set_velocity_inport;

	WriteDataPort<Jointd>  current_position_outport;
	WriteDataPort<Jointd>  current_velocity_outport;

	Method<void(Jointd)> setMaxVelocity;
	Method<void(float)> setMaxVelocityFloat;
	Method<void(Jointd)> setMaxAcceleration;
	Method<void(float)> setMaxAccelerationFloat;

	OrocosRTTArmDriverInterface(std::string name) : TaskContext(name),
			set_position_inport("SetPositionPort"),
			set_velocity_inport("SetVelocityPort"),
			current_position_outport("CurrentPositionPort"),
			current_velocity_outport("CurrentVelocityPort"),
			setMaxVelocity("setMaxVelocity",
			        &OrocosRTTArmDriverInterface::setMaxVelocityF,
			        this),
			setMaxVelocityFloat("setMaxVelocityFloat",
					&OrocosRTTArmDriverInterface::setMaxVelocityFloatF,
			        this),
			setMaxAcceleration("setMaxAcceleration",
					 &OrocosRTTArmDriverInterface::setMaxAccelerationF,
					 this),
			setMaxAccelerationFloat("setMaxAccelerationFloat",
					 &OrocosRTTArmDriverInterface::setMaxAccelerationFloatF,
			         this)
	{
		this->ports()->addEventPort(&set_position_inport);
		this->ports()->addEventPort(&set_velocity_inport);
		this->ports()->addPort(&current_position_outport);
		this->ports()->addPort(&current_velocity_outport);

		this->methods()->addMethod( &setMaxVelocity,
		                "Setting maximal velocity of joints.", "Jointd", "maximal velocity in rad per second");
		this->methods()->addMethod( &setMaxVelocityFloat,
			            "Setting maximal velocity of joints.", "float", "maximal velocity in rad per second");
		this->methods()->addMethod( &setMaxAcceleration,
			            "Setting maximal acceleration of joints.", "Jointd", "maximal acceleration in rad per second squared");
		this->methods()->addMethod( &setMaxAccelerationFloat,
			            "Setting maximal acceleration of joints.", "float", "maximal acceleration in rad per second squared");

	}
	~OrocosRTTArmDriverInterface() {};

private:
	virtual void setMaxVelocityF(Jointd radpersec) = 0;
	virtual void setMaxVelocityFloatF(float radpersec) = 0;
	virtual void setMaxAccelerationF(Jointd radpersec) = 0;
	virtual void setMaxAccelerationFloatF(float radpersec) = 0;


};


#endif /* OROCOSRTTARMDRIVERINTERFACE_H_ */
