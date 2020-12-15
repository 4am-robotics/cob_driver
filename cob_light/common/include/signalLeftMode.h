#ifndef SIGNAL_LEFT_H
#define SIGNAL_LEFT_H

#include <vector>

#include "mode.h"

class SignalLeft : public Mode
{
public:
  SignalLeft(color::rgba color, uint16_t nLeds, uint8_t priority = 0, double freq = 0.25, uint8_t pulses = 0,
             double timeout = 0)
    : Mode(priority, freq, pulses, timeout), _num_leds(nLeds), _toggle(false), _timer_inc(0), leds_margin(5), leds_margin_left(7)
  {
    _color = color;
    if (_pulses != 0)
    {
      _pulses *= 2;
      _pulses += 1;
    }
    _inc = (1. / UPDATE_RATE_HZ) * _freq;
  }

  virtual void execute()
  {
    std::vector<color::rgba> cols;
    color::rgba turnedOff;
    turnedOff.a = 0;

    for (uint16_t i = 0; i < (_num_leds / 2) + leds_margin; ++i)
    {
      cols.push_back(turnedOff);
    }
    for (uint16_t i = (_num_leds / 2) + leds_margin; i < _num_leds - leds_margin_left; ++i)
    {
      cols.push_back(_color);
    }

    if (_timer_inc >= 1.0)
    {
      for(color::rgba& color : cols)
      {
        color.a = color.a * (int)_toggle;
      }
      _pulsed++;
      _toggle = !_toggle;
      m_sigColorsReady(cols);
      _timer_inc = 0.0;
    }
    else
    {
      _timer_inc += _inc;
    }
  }

  std::string getName()
  {
    return std::string("SignalLeft");
  }

private:
  uint16_t _num_leds;
  bool _toggle;
  double _timer_inc;
  double _inc;
  uint8_t leds_margin;
  uint8_t leds_margin_left;
};

#endif  // !SIGNAL_LEFT_H
