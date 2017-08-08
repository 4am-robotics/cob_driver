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
