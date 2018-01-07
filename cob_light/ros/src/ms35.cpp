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
 

#include <ms35.h>
#include <ros/ros.h>
#include <boost/crc.hpp>
#include <boost/cstdint.hpp>
#include <boost/integer.hpp>

MS35::MS35(SerialIO* serialIO)
{
  _serialIO = serialIO;
}

MS35::~MS35()
{
}

bool MS35::init()
{
  bool ret = false;
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
  if(sendData(init_buf, 18) == 18)
    ret = true;
  return ret;
}

unsigned short int MS35::getChecksum(const char* data, size_t len)
{
  unsigned int ret;
  boost::crc_16_type checksum_agent;
  checksum_agent.process_bytes(data, len);
  return checksum_agent.checksum();
}

int MS35::sendData(const char* data, size_t len)
{
  int bytes_wrote = 0;

  bytes_wrote = _serialIO->sendData(data, len);

  if(bytes_wrote == -1)
    ROS_WARN("Can not write to serial port. Port closed!");
  else
    ROS_DEBUG("Wrote [%s] with %i bytes from %lu bytes", data, bytes_wrote, len);

  return bytes_wrote;
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

  //shift values into 8 bit rgb space
  color.r = fabs(color.r * 255);
  color.g = fabs(color.g * 255);
  color.b = fabs(color.b * 255);

  buffer[0] = 0x01; buffer[1] = 0x00;
  buffer[2] = (int)color.r; buffer[3]=(int)color.g; buffer[4]=(int)color.b;
  buffer[5] = 0x00; buffer[6]=0x00;

  unsigned short int crc = getChecksum(buffer, 7);
  buffer[7] = ((char*)&crc)[1];
  buffer[8] = ((char*)&crc)[0];
  
  if(sendData(buffer, PACKAGE_SIZE) == PACKAGE_SIZE)
    m_sigColorSet(color_tmp);
  else
    ROS_ERROR("Could not write to serial port");
}
