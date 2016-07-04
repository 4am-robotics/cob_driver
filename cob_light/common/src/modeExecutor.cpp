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
	_colorO->signalColorSet()->connect(boost::bind(&ModeExecutor::onColorSetReceived, this, _1));
}

ModeExecutor::~ModeExecutor()
{
}

uint64_t ModeExecutor::execute(cob_light::LightMode requestedMode)
{
	boost::shared_ptr<Mode> mode = ModeFactory::create(requestedMode, _colorO);
	// check if mode was correctly created
	if(mode)
		return execute(mode);
}

uint64_t ModeExecutor::execute(boost::shared_ptr<Mode> mode)
{
	uint64_t u_id;

	// check if modes allready executing
	if(_mapActiveModes.size() > 0)
	{
		// check if mode with lower prio is running
		if(_mapActiveModes.begin()->first < mode->getPriority())
		{
			ROS_DEBUG("Pause mode: %i with prio %i",
				ModeFactory::type(_mapActiveModes.begin()->second.get()), _mapActiveModes.begin()->second->getPriority());
			_mapActiveModes.begin()->second->pause();
		}
		else
		{
			std::map<int, boost::shared_ptr<Mode>, std::greater<int> >::iterator itr;
			itr = _mapActiveModes.find(mode->getPriority());
			if(itr != _mapActiveModes.end())
			{
				ROS_DEBUG("Stopping mode: %i with prio %i",
					ModeFactory::type(itr->second.get()), itr->second->getPriority());
				itr->second->stop();
				_mapActiveModes.erase(itr);
			}
		}
	}
	mode->signalColorReady()->connect(boost::bind(&IColorO::setColor, _colorO, _1));
	mode->signalColorsReady()->connect(boost::bind(&IColorO::setColorMulti, _colorO, _1));
	mode->signalModeFinished()->connect(boost::bind(&ModeExecutor::onModeFinishedReceived, this, _1));
	mode->setActualColor(_activeColor);
	ROS_DEBUG("Attaching Mode %i with prio: %i freq: %f timeout: %f pulses: %i ",
		ModeFactory::type(mode.get()), mode->getPriority(), mode->getFrequency(), mode->getTimeout(), mode->getPulses());
	_mapActiveModes.insert(std::pair<int, boost::shared_ptr<Mode> >(mode->getPriority(), mode));

	if(!_mapActiveModes.begin()->second->isRunning())
	{
		ROS_DEBUG("Executing Mode %i with prio: %i freq: %f timeout: %f pulses: %i ",
			ModeFactory::type(mode.get()), mode->getPriority(), mode->getFrequency(), mode->getTimeout(), mode->getPulses());
		_mapActiveModes.begin()->second->start();
	}
	Mode* ptr = mode.get();
	u_id = reinterpret_cast<uint64_t>( ptr );
	return u_id;
}

void ModeExecutor::pause()
{
	if(_mapActiveModes.size() > 0)
	{
		_mapActiveModes.begin()->second->pause();
	}
}

void ModeExecutor::resume()
{
	if(_mapActiveModes.size() > 0 && !_mapActiveModes.begin()->second->isRunning())
		_mapActiveModes.begin()->second->start();
}

void ModeExecutor::stop()
{
	if(_mapActiveModes.size() > 0)
	{
		std::map<int, boost::shared_ptr<Mode>, std::greater<int> >::iterator itr;
		for(itr = _mapActiveModes.begin(); itr != _mapActiveModes.end(); itr++)
		{
			itr->second->stop();
		}
		_mapActiveModes.clear();
	}
}

bool ModeExecutor::stop(uint64_t uId)
{
	bool ret = false;
	if(_mapActiveModes.size() > 0)
	{
		std::map<int, boost::shared_ptr<Mode>, std::greater<int> >::iterator itr;
		for(itr = _mapActiveModes.begin(); itr != _mapActiveModes.end(); itr++)
		{
			uint64_t uid = reinterpret_cast<uint64_t>(itr->second.get());
			if(uid == uId)
			{
				ROS_DEBUG("Stopping mode: %i with prio %i",
					ModeFactory::type(itr->second.get()), itr->second->getPriority());
				itr->second->stop();
				_mapActiveModes.erase(itr);

				if(_mapActiveModes.size() > 0)
				{
					if(!_mapActiveModes.begin()->second->isRunning())
					{
						ROS_DEBUG("Resume mode: %i with prio %i",
							ModeFactory::type(_mapActiveModes.begin()->second.get()), _mapActiveModes.begin()->second->getPriority());
						_mapActiveModes.begin()->second->start();
					}
				}
				ret = true;
				break;
			}
		}
	}
	return ret;
}
void ModeExecutor::onModeFinishedReceived(int prio)
{
	//check if finished mode is the current active
	if(_mapActiveModes.begin()->first == prio)
	{
		//erase mode from map and exec mode with next lower prio
		_mapActiveModes.erase(prio);
		if(_mapActiveModes.size() > 0)
		{
			ROS_DEBUG("Resume mode: %i with prio %i",
				ModeFactory::type(_mapActiveModes.begin()->second.get()), _mapActiveModes.begin()->second->getPriority());
			_mapActiveModes.begin()->second->start();
		}
	}
	//finished mode is not the current executing one (this should never happen)
	else
	{
		ROS_WARN("Mode finished which should't be executed");
		_mapActiveModes.erase(prio);
	}
}

void ModeExecutor::onColorSetReceived(color::rgba color)
{
  _activeColor = color;
}

int ModeExecutor::getExecutingMode()
{
	if(_mapActiveModes.size() > 0)
		return ModeFactory::type(_mapActiveModes.begin()->second.get());
	else
		return ModeFactory::type(NULL);
}

int ModeExecutor::getExecutingPriority()
{
	if(_mapActiveModes.size()>0)
		return _mapActiveModes.begin()->second->getPriority();
	else
		return default_priority;
}

uint64_t ModeExecutor::getExecutingUId()
{
	if(_mapActiveModes.size()>0)
		return reinterpret_cast<uint64_t>(_mapActiveModes.begin()->second.get());
	else
		return 0;
}

void ModeExecutor::setDefaultPriority(int priority)
{
	default_priority = priority;
}
