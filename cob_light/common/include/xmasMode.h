#ifndef XMASMODE_H
#define XMASMODE_H

#include <mode.h>
#include <algorithm>

class XMasMode : public Mode
{
public:
   XMasMode(size_t num_leds, int priority = 0, double freq = 5, int pulses = 0, double timeout = 0)
    :Mode(priority, freq, pulses, timeout), _toggle(false), _timer_inc(0), _num_leds(num_leds)
  {
    _colors.resize(num_leds);
    for(int i = 0; i < num_leds; i++)
    {
      if(i%2 == 0)
      {
        _colors.at(i).r = 1.0; _colors.at(i).g = 0.0; _colors.at(i).b = 0.0; _colors.at(i).a = 1.0;
      }
      else
        _colors.at(i).r = 1.0; _colors.at(i).g = 1.0; _colors.at(i).b = 1.0; _colors.at(i).a = 1.0;
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
};

#endif
