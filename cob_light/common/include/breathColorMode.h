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

#ifndef BREATHCOLORMODE_H
#define BREATHCOLORMODE_H

#include <mode.h>

class BreathColorMode : public Mode
{
public:
	BreathColorMode(color::rgba color, int priority = 0, double freq = 20, int pulses = 0, double timeout = 0)
		:Mode(priority, freq, pulses, timeout), _timer_inc(0.0)
	{
		_color = color;
		h = 0.0;
	}

	void execute()
	{
		float r = 0;
		float g = 0;
		float b = 0;
		
		color::Color::hsv2rgb(h, 1.0, 1.0, r, g, b);
		
		h += 0.001;
		if(h > 1.0) h = 0.0;

		//double fV = (exp(sin(_timer_inc))-1.0/M_E)*(1.000/(M_E-1.0/M_E));
		double fV = (exp(sin(_timer_inc))-0.36787944)*0.42545906411;
		
		_timer_inc += 0.05;
		if(_timer_inc >= M_PI*2)
		{
		 	_timer_inc = 0.0;
		 	_pulsed++;
		}

		color::rgba col;
		col.r = r;
		col.g = g;
		col.b = b;
		col.a = fV;
		
		m_sigColorReady(col);
	}

	std::string getName(){ return std::string("BreathColorMode"); }

private:
	double _timer_inc;
	float h;
};

#endif