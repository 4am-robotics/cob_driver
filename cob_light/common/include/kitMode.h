/*
 * Copyright 2021 Mojin Robotics GmbH https://www.mojin-robotics.de
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


#ifndef KITMODE_H
#define KITMODE_H

#include <mode.h>

class KitMode : public Mode
{
public:
    KitMode(color::rgba color, size_t num_leds, int priority = 0, double freq = 0.25, int pulses = 0, double timeout = 0)
        :Mode(priority, freq, pulses, timeout), _num_leds(num_leds)
    {
      _inc = (1. / UPDATE_RATE_HZ) * _freq;

      _color = color;
      _colors.assign(num_leds, color::rgba());
      _colors[0] = color;
      _pos = 0;
      _dir = 1;
      _fade_fac = 0.8;
    }

    void execute()
    {
      if(_timer_inc >= 1.0)
      {
        for(int i = 0; i < _num_leds; i++)
        {

          _colors[i].a = _colors[i].a * _fade_fac;
        }
        for(int j = 0; j < 5; j++)
        {
          _pos += _dir;
          if(_pos == -1)
          {
            _pos = 0;
            _dir = 1;
          }
          else if(_pos == _num_leds)
          {
            _pos = _num_leds-1;
            _dir = -1;
          }

          _colors[_pos] = _color;

        }
        m_sigColorsReady(_colors);

        _timer_inc = 0.0;
      }
      else
        _timer_inc += _inc;

    }

    std::string getName(){ return std::string("KitMode"); }

private:
  double _timer_inc;
	double _inc;
  double _pos;
  double _dir;
  int _num_leds;
  double _fade_fac;
};

#endif
