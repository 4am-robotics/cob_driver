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

#ifndef FADECOLORMODE_H
#define FADECOLORMODE_H

#include <mode.h>

class FadeColorMode : public Mode
{
public:
	FadeColorMode(color::rgba color, int priority = 0, double freq = 25, int pulses = 0, double timeout = 0)
		:Mode(priority, freq, pulses, timeout)
	{
		_color = color;

		doOnce = true;
		h = 0.0;
		h_s = 0.0;
		h_t = 0.0;
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

		h += 0.0025;
		h_t += 0.0025;
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
};

#endif