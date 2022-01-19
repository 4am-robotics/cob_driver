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



#ifndef TURNINDICATORMODE_H
#define TURNINDICATORMODE_H

#include <mode.h>

class TurnIndicatorMode : public Mode
{
public:
    TurnIndicatorMode(color::rgba color, size_t num_leds, int dir, int priority = 0, double freq = 0.25, int pulses = 0, double timeout = 0)
        :Mode(priority, freq, pulses, timeout), _num_leds(num_leds)
    {
      _inc = (1. / UPDATE_RATE_HZ) * _freq;

      _color = color;
      _colors.assign(_num_leds, color::rgba());
      _colors[0] = color;
      _pos = 0;
      _dir = dir;
      if(_num_leds%2 == 0)
      {
        _center_pos = _num_leds/2 - 1;
        if(_dir < 0)
        _center_pos--;
      }
      else
        _center_pos = _num_leds/2;

      _pos = _center_pos;
      _keep_counter = 20;
    }

    void execute()
    {
      if(_timer_inc >= 1.0)
      {
        //if we exeeded the array keep the colors for a few steps
        //and reset afterwards
        if(_pos == -1 || _pos == _num_leds)
        {
          _keep_counter--;
          if(_keep_counter == 0)
          {
            _keep_counter = 20;
            _pos = _center_pos;
            _colors.assign(_num_leds, color::rgba());
          }
        }
        else
        {
          _colors[_pos] = _color;
          _pos += _dir;
        }

        m_sigColorsReady(_colors);

        _timer_inc = 0.0;
      }
      else
        _timer_inc += _inc;
    }

    std::string getName(){ return std::string("TurnIndicatorMode"); }

private:
  double _timer_inc;
	double _inc;
  double _pos;
  double _dir;
  int _keep_counter;
  int _center_pos;
  int _num_leds;
};

#endif
