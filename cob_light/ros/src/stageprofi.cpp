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

#include <stageprofi.h>
#include <ros/ros.h>
#include <boost/cstdint.hpp>
#include <boost/integer.hpp>
#include <algorithm>

StageProfi::StageProfi(SerialIO* serialIO, unsigned int leds, int led_offset)
{
  _serialIO = serialIO;
  _num_leds = leds;
  _led_offset = led_offset;
}

StageProfi::~StageProfi()
{
}

bool StageProfi::init()
{
  bool ret = false;
  const char init_data[] = { 'C', '?' };
  int init_len = sizeof(init_data) / sizeof(init_data[0]);

  char init_buf[2];

  memcpy(&init_buf, init_data, init_len);
  if (_serialIO->sendData(init_buf, 2) == 2)
  {
    std::string rec;
    if(_serialIO->readData(rec, 1) == 1)
      ret = true;
  }
  else
    ROS_ERROR("Sending init command to stageprofi failed");
  return ret;
}

void StageProfi::setColor(color::rgba color)
{
  color::rgba color_tmp = color;

  color_tmp.r *= color.a;
  color_tmp.g *= color.a;
  color_tmp.b *= color.a;

  color_tmp.r = fabs(color_tmp.r * 255);
  color_tmp.g = fabs(color_tmp.g * 255);
  color_tmp.b = fabs(color_tmp.b * 255);

  unsigned int num_channels = _num_leds * 3;
  char channelbuffer[num_channels];

  for (int i = 0; i < _num_leds; i++)
  {
    channelbuffer[i * 3] = (int) color_tmp.r;
    channelbuffer[i * 3 + 1] = (int) color_tmp.g;
    channelbuffer[i * 3 + 2] = (int) color_tmp.b;
  }

  uint16_t index = 0;
  while (index < num_channels)
  {
    unsigned int size = std::min((unsigned int) MAX_CHANNELS,
        num_channels - index);
    if (sendDMX(index, channelbuffer + index, size))
      index += size;
    else
    {
      ROS_ERROR("Sending color to stageprofi failed");
      this->recover();
      break;
    }
  }
  m_sigColorSet(color);
}

void StageProfi::setColorMulti(std::vector<color::rgba> &colors)
{
  color::rgba color_tmp;
  unsigned int num_channels = _num_leds * 3;
  char channelbuffer[num_channels];

  std::vector<color::rgba> color_out = colors;
  //shift lex index by offset
  if(_led_offset != 0)
  {
    if(_led_offset > 0)
      std::rotate(color_out.begin(), color_out.begin()+_led_offset, color_out.end());
    else
      std::rotate(color_out.begin(), color_out.begin()+color_out.size()+_led_offset, color_out.end());
  }

  for (int i = 0; i < _num_leds || i < colors.size(); i++)
  {
    color_tmp.r = color_out[i].r * color_out[i].a;
    color_tmp.g = color_out[i].g * color_out[i].a;
    color_tmp.b = color_out[i].b * color_out[i].a;

    color_tmp.r = fabs(color_tmp.r * 255);
    color_tmp.g = fabs(color_tmp.g * 255);
    color_tmp.b = fabs(color_tmp.b * 255);

    channelbuffer[i * 3] = (int) color_tmp.r;
    channelbuffer[i * 3 + 1] = (int) color_tmp.g;
    channelbuffer[i * 3 + 2] = (int) color_tmp.b;
  }

  uint16_t index = 0;
  while (index < num_channels)
  {
    unsigned int size = std::min((unsigned int) MAX_CHANNELS,
        num_channels - index);
    if (sendDMX(index, channelbuffer + index, size))
      index += size;
    else
    {
      ROS_ERROR("Sending color to stageprofi failed");
      this->recover();
      break;
    }
  }
  m_sigColorSet(colors[0]);
}

bool StageProfi::sendDMX(uint16_t start, const char* buf, unsigned int length)
{
  bool ret = false;
  std::string recv;
  char msg[MAX_CHANNELS + HEADER_SIZE];

  unsigned int len = std::min((unsigned int) MAX_CHANNELS, length);

  msg[0] = 0xFF;
  msg[1] = start & 0xFF;
  msg[2] = (start >> 8) & 0xFF;
  msg[3] = len;

  memcpy(msg + HEADER_SIZE, buf, len);

  const int bytes_to_send = len + HEADER_SIZE;

  //send color command to controller
  ret = _serialIO->sendData(msg, bytes_to_send) == bytes_to_send;
  //receive ack
  ret &= _serialIO->readData(recv, 1) == 1;
  return ret;
}

bool StageProfi::recover()
{
  ROS_WARN("Trying to recover stagedriver");
  if(_serialIO->recover() && this->init())
    return true;
  else
    return false;
}
