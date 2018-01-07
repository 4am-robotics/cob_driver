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


#ifndef GLOWCOLORMODE_H
#define GLOWCOLORMODE_H

#include <mode.h>

class GlowColorMode : public Mode
{
public:
    GlowColorMode(color::rgba color, int priority = 0, double freq = 0.25, int pulses = 0, double timeout = 0)
        :Mode(priority, freq, pulses, timeout), _timer_inc(0.0)
    {
        _color = color;
        h = 0.0;
        _inc = (1. / UPDATE_RATE_HZ) * _freq;
        h_inc=0;
        color::Color::rgb2hsv(color.r, color.g, color.b, h, s, v);
    }

    void execute()
    {
        static int sign = 1;
        float r = 0;
        float g = 0;
        float b = 0;

        if(_timer_inc >= 1.0)
        {
            double tmp = h;

            h_inc += 0.001 * sign;
            if(h_inc >= 0.01 || h_inc <= -0.01)
            {
                sign *= -1;
                _pulsed++;
            }
            h += h_inc;
            if( h < 0)
                h = 1 + h;
            
            //double fV = (exp(sin(_timer_inc))-1.0/M_E)*(1.000/(M_E-1.0/M_E));
            double fV = (exp(sin( (M_PI/2)+h_inc*60 ))-0.36787944)*0.42545906411;

            color::Color::hsv2rgb(h, s, v, r, g, b);

            color::rgba col;
            col.r = r;
            col.g = g;
            col.b = b;
            col.a = fV;

            _timer_inc = 0.0;
            m_sigColorReady(col);
            h = tmp;
        }
        else
            _timer_inc += _inc;
    }

    std::string getName(){ return std::string("GlowColorMode"); }

private:
    double _timer_inc;
    double _inc;
    float h, s, v;
    double h_inc;
};

#endif
