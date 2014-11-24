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

StageProfi::StageProfi(SerialIO* serialIO)
{
  _serialIO = serialIO;
  const char init_data[] = { 'C', '?' };
  int init_len = sizeof(init_data) / sizeof(init_data[0]);

  char init_buf[2];

  memcpy(&init_buf, init_data, init_len);
  sendData(init_buf, 2);
}

StageProfi::~StageProfi()
{
}

int StageProfi::sendData(const char* data, size_t len)
{
  int ret = -1;
  int bytes_wrote = 0;

  bytes_wrote = _serialIO->sendData(data, len);
  if(bytes_wrote == -1)
  {
    ROS_WARN("Can not write to serial port. Port closed!");
    ret = -1;
  }
  else
  {
    std::string recv;
    ROS_DEBUG("Receiving");
    int byte_recv = _serialIO->readData(recv, 1);
    ROS_DEBUG_STREAM("Received "<<byte_recv<<" bytes after color set: "<<recv);
    ret = 1;
  }
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

  buffer[0] = 0xff; //array command
  buffer[1] = 0x00; //start channel low byte
  buffer[2] = 0x00; //start channel high byte
  buffer[3] = static_cast<char>(MAX_NUM_LEDS); //num channels to set
  
  for(size_t i=0; i<MAX_NUM_LEDS;++i)
  {
    actual_channel = i*3+4;
    buffer[actual_channel] = (int)color_tmp.r;
    buffer[actual_channel+1] = (int)color_tmp.g;
    buffer[actual_channel+2] = (int)color_tmp.b;
  }

  if(sendData(buffer, MAX_NUM_LEDS*3+4))
      m_sigColorSet(color);

    //char check_command[] = { 'C', '0', '0', '0', '?' };
    //sendData(check_command, 5);
  }
}

void StageProfi::setColorMulti(std::vector<color::rgba> &colors)
{
  color::rgba color_tmp = color;

  
  buffer[0] = 0xff; //array command
  buffer[1] = 0x00; //start channel low byte
  buffer[2] = 0x00; //start channel high byte
  buffer[3] = static_cast<char>(colors.size()); //num channels to set
  
  for(size_t i=0; i<colors.size(); ++i)
  {
    color_tmp.r *= colors[i].a;
    color_tmp.g *= colors[i].a;
    color_tmp.b *= colors[i].a;

    color_tmp.r = fabs(color_tmp.r * 255);
    color_tmp.g = fabs(color_tmp.g * 255);
    color_tmp.b = fabs(color_tmp.b * 255);

    actual_channel = i*3+4;
    buffer[actual_channel] = (int)color_tmp.r;
    buffer[actual_channel+1] = (int)color_tmp.g;
    buffer[actual_channel+2] = (int)color_tmp.b;
  }

  if(sendData(buffer, MAX_NUM_LEDS*3+4))
      m_sigColorSet(color);
}
