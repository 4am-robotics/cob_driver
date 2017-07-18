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

struct rgb
{
  rgb(): r(0.0), g(0.0), b(0.0){}
  float r;
  float g;
  float b;
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

    static float linearInterpolate(float a, float b, float t)
    {
      return a * (1 - t) + b * t;
    }

    static color::rgba interpolateColor(color::rgba start, color::rgba goal, float t)
    {
      color::hsv ca;
      color::hsv cb;
      color::hsv cr;
      color::rgba a, b;
      a = start;
      b = goal;

      a.r *= a.a;
      a.g *= a.a;
      a.b *= a.a;
      b.r *= b.a;
      b.g *= b.a;
      b.b *= b.a;
      color::Color::rgb2hsv(a.r, a.g, a.b, ca.h, ca.s, ca.v);
      color::Color::rgb2hsv(b.r, b.g, b.b, cb.h, cb.s, cb.v);

      cr.h = linearInterpolate(ca.h, cb.h, t);
      cr.s = linearInterpolate(ca.s, cb.s, t);
      cr.v = linearInterpolate(ca.v, cb.v, t);

      color::rgba result;
      color::Color::hsv2rgb(cr.h, cr.s, cr.v, result.r, result.g, result.b);
      result.a = 1.0;

      return result;
    }
};
}
#endif
