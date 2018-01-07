/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


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
	else
		return 0;
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
