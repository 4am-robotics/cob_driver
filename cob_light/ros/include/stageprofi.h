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


#ifndef COB_DRIVER_STAGE_PROFI_H
#define COB_DRIVER_STAGE_PROFI_H

#include <iColorO.h>

#include <serialIO.h>
#include <colorUtils.h>
#include <sstream>

class StageProfi : public IColorO
{
public:
  StageProfi(SerialIO* serialIO, unsigned int leds, int led_offset);
  virtual ~StageProfi();

  bool init();

  void setColor(color::rgba color);
  void setColorMulti(std::vector<color::rgba> &colors);

private:
  SerialIO* _serialIO;
  std::stringstream _ssOut;
  int _led_offset;
  static const unsigned int HEADER_SIZE = 4;
  static const unsigned int MAX_CHANNELS = 255;

  bool recover();
  bool sendDMX(uint16_t start, const char* buf, unsigned int length);
};

#endif
