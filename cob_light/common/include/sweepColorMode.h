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


#ifndef SWEEPCOLORMODE_H
#define SWEEPCOLORMODE_H

#include <mode.h>
#include <algorithm>

class SweepColorMode : public Mode
{
public:
   SweepColorMode(std::vector<color::rgba> colors, size_t num_leds, int priority = 0, double freq = 5, int pulses = 0, double timeout = 0)
    :Mode(priority, freq, pulses, timeout), _toggle(false), _timer_inc(0), _num_leds(num_leds)
  {
    _colors = colors;

    _startcolor.r = 0.0;
    _startcolor.g = 0.4;
    _startcolor.b = 0.28;
    _startcolor.a = 1.0;
    if(_colors.size() == 1)
      _startcolor = _colors[0];
    _colors.resize(_num_leds);
    if(_num_leds%2 == 0)
    {
      _colors[_num_leds/2] = _startcolor;
      _colors[_num_leds/2-1] = _startcolor;
    }
    else
      _colors[_num_leds/2] = _startcolor;
    _pos = 0;

    _inc = (1. / UPDATE_RATE_HZ) * _freq;
  }

  void execute()
  {
    if(_timer_inc >= 1.0)
    {
      for(int i = _num_leds/2; i < _num_leds; i++)
      {
        _colors[i].a *= 0.7;
      }
      for(int i = _num_leds/2-1; i >= 0; i--)
      {
        _colors[i].a *= 0.7;
      }
      _colors[(_num_leds/2)+_pos] = _startcolor;
      _colors[(_num_leds/2)-1-_pos] = _startcolor;

      _pos++;
      if(_pos >= (_num_leds/2))
        _pos = 0;

      _pulsed++;
      m_sigColorsReady(_colors);
      _timer_inc = 0.0;
    }
    else
      _timer_inc += _inc;
  }

  std::string getName(){ return std::string("SweepColorMode"); }

private:
  bool _toggle;
  double _timer_inc;
  double _inc;
  size_t _num_leds;
  int _pos;
  color::rgba _startcolor;
};

#endif
