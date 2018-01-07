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


#ifndef STATICMODE_H
#define STATICMODE_H

#include <mode.h>

class StaticMode : public Mode
{
public:
    StaticMode(color::rgba color, int priority = 0, double freq = 0, int pulses = 0, double timeout = 0)
        :Mode(priority, freq, pulses, timeout), _timer_inc(0)
    {
        _color = color;
        _inc = (1. / UPDATE_RATE_HZ) * _freq;
    }

    void execute()
    {
        if(_timer_inc == 0.0)
            m_sigColorReady(_color);
        if(_timer_inc >= 1.0)
            _timer_inc = 0.0;
        else
            _timer_inc += _inc;
    }

    std::string getName(){ return std::string("StaticMode"); }

private:
    double _timer_inc;
    double _inc;
};

#endif
