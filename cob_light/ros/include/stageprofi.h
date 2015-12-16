/****************************************************************
 *
 * Copyright (c) 2014
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_light
 * Description: Switch robots led color by sending data to
 * the DMX StageProfi
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Thiago de Freitas, email:tdf@ipa.fhg.de
 * Supervised by: Thiago de Freitas, email:tdf@ipa.fhg.de
 *
 * Date of creation: October 2014
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

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
