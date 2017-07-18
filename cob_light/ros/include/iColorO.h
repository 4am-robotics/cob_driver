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


#ifndef ICOLORO_H
#define ICOLORO_H

#include <colorUtils.h>
#include <boost/signals2.hpp>

class IColorO
{
public:
  IColorO() : _initialized(false), _invertMask(0), _num_leds(1){;}
  virtual ~IColorO(){;}

  virtual bool init() = 0;
  virtual void setColor(color::rgba color) = 0;
  virtual void setColorMulti(std::vector<color::rgba> &colors) = 0;

  void setMask(int mask){ _invertMask = mask; }
  void setNumLeds(size_t num_leds){ _num_leds = num_leds; }
  int getNumLeds(){ return _num_leds; }

  boost::signals2::signal<void (color::rgba color)>* signalColorSet(){ return &m_sigColorSet; }
  boost::signals2::signal<void (std::vector<color::rgba> colors) >* signalColorsSet(){ return &m_sigColorsSet; }

protected:
  bool _initialized;
  int _invertMask;
  int _num_leds;
  boost::signals2::signal<void (color::rgba color)> m_sigColorSet;
  boost::signals2::signal<void (std::vector<color::rgba> colors) > m_sigColorsSet;
};

#endif
