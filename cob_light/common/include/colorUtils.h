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

#ifndef COLOR_UTILS_H
#define COLOR_UTILS_H

#include <algorithm>
#include <math.h>

namespace color
{
struct rgba
{
	rgba(): r(0.0), g(0.0), b(0.0), a(0.0) {}
	float r;
	float g;
	float b;
	float a;
};

struct hsv
{
	hsv(): h(0.0), s(0.0), v(0.0) {}
	float h;
	float s;
	float v;
};

class Color
{
public:
	static void rgb2hsv (float r, float g, float b, float &h, float &s, float &v)
	{
		double var_R = r;
		double var_G = g;
		double var_B = b;

		double var_Min = std::min(std::min(var_R,var_G),var_B);
		double var_Max = std::max(std::max(var_R,var_G),var_B);
		double del_Max = var_Max - var_Min;
		v = var_Max;
		if (fabs(del_Max)<0.00001) {
			h = 0;
			s = 0;
		}
		else {
			s = del_Max/var_Max;

			if      ( var_R == var_Max ) h = (var_G - var_B)/del_Max;
			else if ( var_G == var_Max ) h = 2.0 + (var_B - var_R)/del_Max;
			else if ( var_B == var_Max ) h = 4.0 + (var_R - var_G)/del_Max;
			h /= 6.0;

			if ( h < 0 )  h += 1;
			if ( h > 1 )  h -= 1;
		}
	}

	static void hsv2rgb (float h, float s, float v, float &r, float &g, float &b)
	{
		float h1 = h*6; // sector 0 to 5
		int i = floor( h1 );
		float f = h1 - i; // fractional part of h

		float p = v * ( 1 - s );
		float q = v * ( 1 - s * f );
		float t = v * ( 1 - s * ( 1 - f ) );

		if      (i==0) {r = v;  g = t;  b = p;}
		else if (i==1) {r = q;  g = v;  b = p;}
		else if (i==2) {r = p;  g = v;  b = t;}
		else if (i==3) {r = p;  g = q;  b = v;}
		else if (i==4) {r = t;  g = p;  b = v;}
		else if (i==5) {r = v;  g = p;  b = q;}
	}
};
}
#endif