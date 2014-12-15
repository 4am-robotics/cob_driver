/****************************************************************
 *
 * Copyright (c) 2010
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
 * the led-µC over serial connection.
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Benjamin Maidel, email:benjamin.maidel@ipa.fraunhofer.de
 * Supervised by: Benjamin Maidel, email:benjamin.maidel@ipa.fraunhofer.de
 *
 * Date of creation: August 2012
 * ToDo:
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

#include <ms35.h>
#include <ros/ros.h>
#include <boost/crc.hpp>
#include <boost/cstdint.hpp>
#include <boost/integer.hpp>

MS35::MS35(SerialIO* serialIO)
{
  _serialIO = serialIO;
  const char init_data[] = { 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd, 0xfd };
  int init_len = sizeof(init_data) / sizeof(init_data[0]);
  const char startup_data[] = { 0xfd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  int startup_len = sizeof(startup_data) / sizeof(startup_data[0]);
  char init_buf[18];

  //write data until controller is ready to receive valid package
  char tmp = 0xfd;
  std::string recv;
  int bytes_recv;
  do{
    _serialIO->sendData(&tmp, 1);
    bytes_recv = _serialIO->readData(recv, 1);

  }while(bytes_recv <= 0);

  unsigned short int crc = getChecksum(startup_data, startup_len);

  memcpy(&init_buf, init_data, init_len);
  memcpy(&init_buf[init_len], startup_data, startup_len);
  init_buf[init_len+startup_len] = ((char*)&crc)[1];
  init_buf[init_len+startup_len+1] = ((char*)&crc)[0];
  sendData(init_buf, 18);
}

MS35::~MS35()
{
}

unsigned short int MS35::getChecksum(const char* data, size_t len)
{
  unsigned int ret;
  boost::crc_16_type checksum_agent;
  checksum_agent.process_bytes(data, len);
  return checksum_agent.checksum();;
}

int MS35::sendData(const char* data, size_t len)
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
    ROS_DEBUG("Wrote [%s] with %i bytes from %lu bytes", data, bytes_wrote, len);
    //std::string recv;
    //ROS_INFO("Receiving");
    //int byte_recv = _serialIO->readData(recv, 1);
    //ROS_INFO_STREAM("Received "<<byte_recv<<" bytes after color set: "<<recv);
    //ret = 1;
  }
  return ret;
}

void MS35::setColorMulti(std::vector<color::rgba> &colors)
{

}

void MS35::setColor(color::rgba color)
{
  color::rgba color_tmp = color;

  //calculate rgb spektrum for spezific alpha value, because
  //led board is not supporting alpha values
  color.r *= color.a;
  color.g *= color.a;
  color.b *= color.a;

  color.r = fabs(color.r * 255);
  color.g = fabs(color.g * 255);
  color.b = fabs(color.b * 255);

  buffer[0] = 0x01; buffer[1] = 0x00;
  buffer[2] = (int)color.r; buffer[3]=(int)color.g; buffer[4]=(int)color.b;
  buffer[5] = 0x00; buffer[6]=0x00;

  unsigned short int crc = getChecksum(buffer, 7);
  buffer[7] = ((char*)&crc)[1];
  buffer[8] = ((char*)&crc)[0];
  //memcpy(&buffer[7], &crc, sizeof(unsigned short int));

  if(sendData(buffer, PACKAGE_SIZE))
    m_sigColorSet(color_tmp);
}
