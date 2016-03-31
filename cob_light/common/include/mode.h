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

#ifndef MODE_H
#define MODE_H

#include <ros/ros.h>
#include <colorUtils.h>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>

class Mode
{
public:
	Mode(int priority = 0, double freq = 0, int pulses = 0, double timeout = 0)
		: _priority(priority), _freq(freq), _pulses(pulses), _timeout(timeout),
		  _finished(false), _pulsed(0), _isStopRequested(false), _isPauseRequested(false),
		  _isRunning(false){}
	virtual ~Mode(){}

	void start()
	{
		if(_thread == NULL)
			_thread.reset(new boost::thread(&Mode::run, this));
		if(isPauseRequested())
		{
			boost::mutex::scoped_lock lock(_mutex_pause);
			_isPauseRequested = false;
			_cond_pause.notify_one();
		}
		_isRunning = true;
	}

	void stop()
	{
		_mutex.lock();
		_isStopRequested = true;
		_mutex.unlock();
		if(isPauseRequested())
		{
			_isPauseRequested = false;
			boost::mutex::scoped_lock lock(_mutex_pause);
			_cond_pause.notify_one();
		}
		if(_thread != NULL)
		{
			_thread->join();
			_thread.reset();
		}
		_isStopRequested = false;
		_isRunning = false;
	}

	void pause()
	{
		_mutex.lock();
		_isPauseRequested = true;
		_mutex.unlock();
		_isRunning = false;
	}

	virtual void execute() = 0;

	virtual std::string getName() = 0;

	bool finished(){ return _finished; }

	void setPriority(int priority){ _priority = priority; }
	int getPriority(){ return _priority; }

	void setTimeout(double timeout){ _timeout = timeout; }
	double getTimeout(){ return _timeout; }

	void setFrequency(double freq){ _freq = freq; }
	double getFrequency(){ return _freq; }

	void setPulses(int pulses){ _pulses = pulses; }
	int getPulses(){ return _pulses; }

	int pulsed(){ return _pulsed; }

	void setColor(color::rgba color){ _color = color; }
	color::rgba getColor(){ return _color; }

	void setActualColor(color::rgba color){ _actualColor = color; }
	color::rgba getActualColor(){ return _color; }

	bool isRunning(){ return _isRunning; }

	boost::signals2::signal<void (color::rgba color)>* signalColorReady(){ return &m_sigColorReady; }
	boost::signals2::signal<void (std::vector<color::rgba> colors)>* signalColorsReady(){ return &m_sigColorsReady; }
	boost::signals2::signal<void (int)>* signalModeFinished(){ return &m_sigFinished; }

protected:
	int _priority;
	double _freq;
	int _pulses;
	double _timeout;

	bool _finished;
	int _pulsed;

	color::rgba _color;
	std::vector<color::rgba> _colors;
	color::rgba _actualColor;
	color::rgba _init_color;

	static const unsigned int UPDATE_RATE_HZ = 100;

	boost::signals2::signal<void (color::rgba color)> m_sigColorReady;
	boost::signals2::signal<void (std::vector<color::rgba> colors)> m_sigColorsReady;
	boost::signals2::signal<void (int)> m_sigFinished;

private:
	boost::shared_ptr<boost::thread> _thread;
	boost::mutex _mutex;
	boost::mutex _mutex_pause;
	bool _isStopRequested;
	bool _isPauseRequested;
	bool _isRunning;

	boost::condition_variable _cond_pause;

	bool isStopRequested()
	{
		bool ret;
		_mutex.lock();
		ret = _isStopRequested;
		_mutex.unlock();
		return ret;
	}

	bool isPauseRequested()
	{
		bool ret;
		_mutex.lock();
		ret = _isPauseRequested;
		_mutex.unlock();
		return ret;
	}

protected:
	virtual void run()
	{
		ros::Rate r(UPDATE_RATE_HZ);
		if(this->getFrequency() == 0.0)
		  this->setFrequency(1);

		ros::Time timeStart = ros::Time::now();

		while(!isStopRequested() && !ros::isShuttingDown())
		{
			while(isPauseRequested())
			{
				boost::mutex::scoped_lock lock(_mutex_pause);
				_cond_pause.wait(lock);
			}
			this->execute();

			if((this->getPulses() != 0) &&
				(this->getPulses() <= this->pulsed()))
				break;

			if(this->getTimeout() != 0)
			{
				ros::Duration timePassed = ros::Time::now() - timeStart;
				if(timePassed.toSec() >= this->getTimeout())
					break;
			}
			r.sleep();
		}
		ROS_DEBUG("Mode %s finished",this->getName().c_str());
		if(!isStopRequested())
			m_sigFinished(this->getPriority());
	}
};

#endif
