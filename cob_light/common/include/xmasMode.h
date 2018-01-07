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


#ifndef XMASMODE_H
#define XMASMODE_H

#include <mode.h>
#include <algorithm>

class XMasMode : public Mode
{
public:
   XMasMode(size_t num_leds, int priority = 0, double freq = 5, int pulses = 0, double timeout = 0)
    :Mode(priority, freq, pulses, timeout), _toggle(false), _timer_inc(0), _num_leds(num_leds), _chucksize(5)
  {
    _colors.resize(num_leds);
    if(_num_leds >= _chucksize*2)
    {
      int n_chunks = _num_leds / _chucksize + (_num_leds%_chucksize ? 1 : 0);
      int k = 0;
      for(int i = 0; i < n_chunks; i++)
      {
        for(int j = 0; j < _chucksize; j++)
        {
          if(k >= _num_leds)
            break;
          if(i%2 == 0)
          {
            _colors.at(k).r = 1.0; _colors.at(k).g = 0.0; _colors.at(k).b = 0.0; _colors.at(k).a = 1.0;
          }
          else
          {
            _colors.at(k).r = 1.0; _colors.at(k).g = 1.0; _colors.at(k).b = 1.0; _colors.at(k).a = 1.0;
          }
          k++;
        }
      }
    }
    else
    {
      for(int i = 0; i < _num_leds; i++)
      {
        if(i%2==0)
        {
          _colors.at(i).r = 1.0; _colors.at(i).g = 0.0; _colors.at(i).b = 0.0; _colors.at(i).a = 1.0;
        }
        else
        {
          _colors.at(i).r = 1.0; _colors.at(i).g = 1.0; _colors.at(i).b = 1.0; _colors.at(i).a = 1.0;
        }
      }
    }
    if(_pulses != 0)
    {
      _pulses *=2;
      _pulses +=1;
    }
    _inc = (1. / UPDATE_RATE_HZ) * _freq;
  }

  void execute()
  {
    if(_timer_inc >= 1.0)
    {
      if(_num_leds >= _chucksize*2)
        std::rotate(_colors.begin(), _colors.begin()+_chucksize, _colors.end());
      else
        std::rotate(_colors.begin(), _colors.begin()+1, _colors.end());
      _pulsed++;
      m_sigColorsReady(_colors);
      _timer_inc = 0.0;
    }
    else
      _timer_inc += _inc;
  }

  std::string getName(){ return std::string("XMasMode"); }

private:
  bool _toggle;
  double _timer_inc;
  double _inc;
  size_t _num_leds;
  size_t _chucksize;
};

#endif
