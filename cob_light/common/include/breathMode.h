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


#ifndef BREATHMODE_H
#define BREATHMODE_H

#include <mode.h>

class BreathMode : public Mode
{
public:
	BreathMode(color::rgba color, int priority = 0, double freq = 0.25, int pulses = 0, double timeout = 0)
		:Mode(priority, freq, pulses, timeout), _timer_inc(0.0)
	{
		_color = color;
		_init_color = color;
		_inc = ((M_PI*2) / UPDATE_RATE_HZ) * _freq;
	}

	void execute()
	{
		//double fV = (exp(sin(_timer_inc))-1.0/M_E)*(1.000/(M_E-1.0/M_E));
		double fV = (exp(-cos(_timer_inc))-0.36787944)*0.42545906411;

		_timer_inc += _inc;
		if(_timer_inc >= M_PI*2)
		{
		 	_timer_inc = 0.0;
		 	_pulsed++;
		}

		_color.a = fV * _init_color.a;
		_color.a = _color.a > 1 ? 1 : _color.a < 0 ? 0 : _color.a;

		m_sigColorReady(_color);
	}

	std::string getName(){ return std::string("BreathMode"); }

private:
	double _timer_inc;
	double _inc;
};

#endif
