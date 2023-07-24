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

class TurnIndicatorMode : public Mode {
public:
  TurnIndicatorMode(color::rgba color, size_t num_leds, int dir,
                    int priority = 0, double freq = 0.25, int pulses = 0,
                    double timeout = 0)
      : Mode(priority, freq, pulses, timeout), _num_leds(num_leds),
        _timer_inc(0), _toggle(false) {
    _color = color;
    _inc = (1. / UPDATE_RATE_HZ) * _freq;

    _colors.assign(_num_leds, color::rgba());
    _dir = dir;
    // If the direction is -1 (left), then the LED indices are set to to the
    // first and last quarters of the LEDs.
    if (dir == -1) {
      size_t from = 0;
      size_t until = _num_leds / 4;
      for (size_t i = from; i < until; ++i) {
        _led_indices.push_back(i);
      }
      from = 3 * _num_leds / 4;
      until = _num_leds;
      for (size_t i = from; i < until; ++i) {
        _led_indices.push_back(i);
      }
    } else {
      // If the direction is not -1 (right), then the LED indices are set to the
      // second and third quarters of the LEDs.
      size_t from = _num_leds / 4;
      size_t until = 3 * _num_leds / 4;
      for (size_t i = from; i < until; ++i) {
        _led_indices.push_back(i);
      }
    }
  }

  void execute() {
    if (_timer_inc >= 1.0) {
      color::rgba col;
      col.r = _color.r;
      col.g = _color.g;
      col.b = _color.b;
      col.a = _color.a * (int)_toggle;
      _toggle = !_toggle;
      for (size_t i = 0; i < _led_indices.size(); ++i) {
        _colors[_led_indices[i]] = col;
      }
      m_sigColorsReady(_colors);
      _timer_inc = 0.0;
    } else {
      _timer_inc += _inc;
    }
  }

  std::string getName() { return std::string("TurnIndicatorMode"); }

private:
  double _timer_inc;
  double _inc;
  double _dir;
  int _num_leds;
  bool _toggle;
  std::vector<size_t> _led_indices;
};

#endif
