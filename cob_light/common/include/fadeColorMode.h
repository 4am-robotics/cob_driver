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


#ifndef FADECOLORMODE_H
#define FADECOLORMODE_H

#include <mode.h>

class FadeColorMode : public Mode
{
public:
	FadeColorMode(color::rgba color, int priority = 0, double freq = 0.25, int pulses = 0, double timeout = 0)
		:Mode(priority, freq, pulses, timeout)
	{
		_color = color;

		doOnce = true;
		h = 0.0;
		h_s = 0.0;
		h_t = 0.0;

		_inc = (1. / UPDATE_RATE_HZ) * _freq;
	}

	void execute()
	{
		float r = 0;
		float g = 0;
		float b = 0;
		float s, v;

		if(doOnce == true)
		{
			color::Color::rgb2hsv(_color.r, _color.g, _color.b, h, s, v);
			h_s = h;
			h_t = h;
			h_s += 1;
			doOnce = false;
		}

		color::Color::hsv2rgb(h, 1.0, 1.0, r, g, b);

		h += _inc;
		h_t += _inc;
		if(h > 1.0)
			h = 0.0;

		if(h_t >= h_s)
		{
			_pulsed++; h_s+=1;
		}

		color::rgba col;
		col.r = r;
		col.g = g;
		col.b = b;
		col.a = _color.a;

		m_sigColorReady(col);
	}

	std::string getName(){ return std::string("FadeColorMode"); }

private:
	bool doOnce;
	float h;
	float h_s;
	float h_t;
	double _inc;
};

#endif
