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
#include <boost/crc.hpp>
#include <boost/cstdint.hpp>
#include <boost/integer.hpp>

#define MAX_NUM_LEDS 60

StageProfi::StageProfi(SerialIO* serialIO)
{
  _serialIO = serialIO;
  const char init_data[] = { 'C', '?' };
  int init_len = sizeof(init_data) / sizeof(init_data[0]);

  char init_buf[2];

  //write data until controller is ready to receive valid package
  char tmp = 0xfd;
  std::string recv;
  int bytes_recv;
  do{
    _serialIO->sendData(&tmp, 1);
    bytes_recv = _serialIO->readData(recv, 1);

  }while(bytes_recv <= 0);

  memcpy(&init_buf, init_data, init_len);
  sendData(init_buf, 2);
}

StageProfi::~StageProfi()
{
}

unsigned short int StageProfi::getChecksum(const char* data, size_t len)
{
  unsigned int ret;
  boost::crc_16_type checksum_agent;
  checksum_agent.process_bytes(data, len);
  return checksum_agent.checksum();;
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

void StageProfi::updateColorBuffer(float color_value)
{
  int cvalue2 = (static_cast<int>(color_value)-static_cast<int>(color_value)%100)/100;
  int cvalue1 = (static_cast<int>(color_value)%100-static_cast<int>(color_value)%10)/10;
  int cvalue0 = static_cast<int>(color_value)%10;

  buffer[5] = static_cast<char>('0'+cvalue2);
  buffer[6] = static_cast<char>('0'+cvalue1);
  buffer[7] = static_cast<char>('0'+cvalue0);
}

void StageProfi::updateChannelBuffer()
{
  actual_channel = actual_channel+1;

  int ac_ch2 = (static_cast<int>(actual_channel)-static_cast<int>(actual_channel)%100)/100;
  int ac_ch1 = (static_cast<int>(actual_channel)%100-static_cast<int>(actual_channel)%10)/10;
  int ac_ch0 = static_cast<int>(actual_channel)%10;

  buffer[1] = static_cast<char>('0'+ac_ch2);
  buffer[2] = static_cast<char>('0'+ac_ch1);
  buffer[3] = static_cast<char>('0'+ac_ch0);
}

void StageProfi::setColor(color::rgba color)
{
  for(size_t i=0; i<MAX_NUM_LEDS;i++)
  {
    actual_channel = i*3;
    color::rgba color_tmp = color;

    color_tmp.r *= color.a;
    color_tmp.g *= color.a;
    color_tmp.b *= color.a;

    color_tmp.r = fabs(color_tmp.r * 255);
    color_tmp.g = fabs(color_tmp.g * 255);
    color_tmp.b = fabs(color_tmp.b * 255);

    buffer[0] = 'C';

    int ac_ch2 = (static_cast<int>(actual_channel)-static_cast<int>(actual_channel)%100)/100;
    int ac_ch1 = (static_cast<int>(actual_channel)%100-static_cast<int>(actual_channel)%10)/10;
    int ac_ch0 = static_cast<int>(actual_channel)%10;

    buffer[1] = static_cast<char>('0'+ac_ch2);
    buffer[2] = static_cast<char>('0'+ac_ch1);
    buffer[3] = static_cast<char>('0'+ac_ch0);

    buffer[4] = 'L';

    updateColorBuffer(color_tmp.r);

    if(sendData(buffer, PACKAGE_SIZE))
      m_sigColorSet(color);

    updateColorBuffer(color_tmp.g);
    updateChannelBuffer();

    if(sendData(buffer, PACKAGE_SIZE))
      m_sigColorSet(color);

    updateColorBuffer(color_tmp.b);
    updateChannelBuffer();

    if(sendData(buffer, PACKAGE_SIZE))
      m_sigColorSet(color);

    char check_command[] = { 'C', '0', '0', '0', '?' };
    sendData(check_command, 5);
  }
}

void StageProfi::setColorMulti(std::vector<color::rgba> &colors)
{
  for(size_t i=0; i<colors.size();i++)
  {
    actual_channel = i*3;
    color::rgba color_tmp = colors[i];

    colors[i].r *= colors[i].a;
    colors[i].g *= colors[i].a;
    colors[i].b *= colors[i].a;

    colors[i].r = fabs(colors[i].r * 255);
    colors[i].g = fabs(colors[i].g * 255);
    colors[i].b = fabs(colors[i].b * 255);

    buffer[0] = 'C';

    int ac_ch2 = (static_cast<int>(actual_channel)-static_cast<int>(actual_channel)%100)/100;
    int ac_ch1 = (static_cast<int>(actual_channel)%100-static_cast<int>(actual_channel)%10)/10;
    int ac_ch0 = static_cast<int>(actual_channel)%10;

    buffer[1] = static_cast<char>('0'+ac_ch2);
    buffer[2] = static_cast<char>('0'+ac_ch1);
    buffer[3] = static_cast<char>('0'+ac_ch0);

    buffer[4] = 'L';

    updateColorBuffer(colors[i].r);

    if(sendData(buffer, PACKAGE_SIZE))
      m_sigColorSet(color_tmp);

    updateColorBuffer(colors[i].g);
    updateChannelBuffer();

    if(sendData(buffer, PACKAGE_SIZE))
      m_sigColorSet(color_tmp);

    updateColorBuffer(colors[i].b);
    updateChannelBuffer();

    if(sendData(buffer, PACKAGE_SIZE))
      m_sigColorSet(color_tmp);

    char check_command[] = { 'C', '0', '0', '0', '?' };
    sendData(check_command, 5);
  }
}
