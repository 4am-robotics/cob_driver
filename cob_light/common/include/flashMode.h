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


#ifndef FLASHMODE_H
#define FLASHMODE_H

#include <mode.h>

class FlashMode : public Mode
{
public:
	FlashMode(color::rgba color, int priority = 0, double freq = 0.25, int pulses = 0, double timeout = 0)
		:Mode(priority, freq, pulses, timeout), _toggle(false), _timer_inc(0)
	{
		_color = color;
		if(_pulses != 0)
		{
			_pulses *=2;
			_pulses +=1;
		}
		_inc = (1. / UPDATE_RATE_HZ) * _freq;
	}

	void execute()
	{
		color::rgba col;
		col.r = _color.r;
		col.g = _color.g;
		col.b = _color.b;
		if(_timer_inc >= 1.0)
		{
		  col.a = _color.a * (int)_toggle;
		  _pulsed++;
		  _toggle = !_toggle;
		  m_sigColorReady(col);
		  _timer_inc = 0.0;
		}
		else
		  _timer_inc += _inc;
	}

	std::string getName(){ return std::string("FlashMode"); }

private:
	bool _toggle;
	double _timer_inc;
	double _inc;
};

#endif
