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
 * ROS package name: cob_light
 * Description: Switch robots led color by sending data to
 * the led-µC over serial connection.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Benjamin Maidel, email:benjamin.maidel@ipa.fraunhofer.de
 * Supervised by: Benjamin Maidel, email:benjamin.maidel@ipa.fraunhofer.de
 *
 * Date of creation: August 2012
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

#include <modeExecutor.h>
#include <ros/ros.h>

ModeExecutor::ModeExecutor(IColorO* colorO)
: _stopRequested(false), default_priority(0)
{
	_colorO = colorO;
	_activeMode = NULL;
}

ModeExecutor::~ModeExecutor()
{
	
}

void ModeExecutor::execute(cob_light::LightMode requestedMode)
{
	Mode* mode = ModeFactory::create(requestedMode);
	// check if mode was correctly created
	if(mode != NULL)
		execute(mode);
}

void ModeExecutor::execute(Mode* mode)
{
	// check if a mode is already executed
	if(_activeMode != NULL)
	{
		// check if priority from requested mode is higher or the same
		if(_activeMode->getPriority() <= mode->getPriority())
		{
			stop();
			_activeMode = mode;
			_activeMode->signalColorReady()->connect(boost::bind(&IColorO::setColor, _colorO, _1));
			_thread_ptr.reset(new boost::thread(boost::lambda::bind(&ModeExecutor::run, this)));
			ROS_INFO("Executing new mode: %s",_activeMode->getName().c_str() );
			ROS_DEBUG("Executing Mode %i with prio: %i freq: %f timeout: %f pulses: %i ",
				ModeFactory::type(mode), mode->getPriority(), mode->getFrequency(), mode->getTimeout(), mode->getPulses());
		}
		else
			ROS_DEBUG("Mode with higher priority is allready executing");
	}
	else
	{
		_activeMode = mode;
		_activeMode->signalColorReady()->connect(boost::bind(&IColorO::setColor, _colorO, _1));
		_thread_ptr.reset(new boost::thread(boost::lambda::bind(&ModeExecutor::run, this)));
		ROS_INFO("Executing new mode: %s",_activeMode->getName().c_str() );
		ROS_DEBUG("Executing Mode %i with prio: %i freq: %f timeout: %f pulses: %i ",
				ModeFactory::type(mode), mode->getPriority(), mode->getFrequency(), mode->getTimeout(), mode->getPulses());
	}

}

void ModeExecutor::run()
{
	ros::Rate r(10);
	if(_activeMode->getFrequency() != 0.0)
		r = ros::Rate(_activeMode->getFrequency());
	else
		_activeMode->setFrequency(10);

	ros::Time timeStart = ros::Time::now();

	while(!isStopRequested() && !ros::isShuttingDown())
	{
		_activeMode->execute();

		if((_activeMode->getPulses() != 0) && 
			(_activeMode->getPulses() <= _activeMode->pulsed()))
			break;

		if(_activeMode->getTimeout() != 0)
		{
			ros::Duration timePassed = ros::Time::now() - timeStart;
			if(timePassed.toSec() >= _activeMode->getTimeout())
				break;
		}
		r.sleep();
	}
	ROS_INFO("Mode %s finished",_activeMode->getName().c_str());
	delete _activeMode;
	_activeMode = NULL;
}

void ModeExecutor::stop()
{
	_mutex.lock();
	_stopRequested = true;
	_mutex.unlock();

	if (_thread_ptr)
		_thread_ptr->join();

	_mutex.lock();
	_stopRequested = false;
	_mutex.unlock();

}

bool ModeExecutor::isStopRequested()
{
	bool ret;
	_mutex.lock();
	ret =_stopRequested;
	_mutex.unlock();
	return ret;
}

int ModeExecutor::getExecutingMode()
{
	return ModeFactory::type(_activeMode);
}

int ModeExecutor::getExecutingPriority()
{
	if(_activeMode != NULL)
		return _activeMode->getPriority();
	else
		return default_priority;
}

void ModeExecutor::setDefaultPriority(int priority)
{
	default_priority = priority;
}