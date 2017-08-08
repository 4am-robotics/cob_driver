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
 

#include <colorO.h>
#include <ros/ros.h>

ColorO::ColorO(SerialIO* serialIO)
{
  _serialIO = serialIO;
}

ColorO::~ColorO()
{
}

bool ColorO::init()
{
  return true;
}

void ColorO::setColorMulti(std::vector<color::rgba> &colors)
{

}

void ColorO::setColor(color::rgba color)
{
  int bytes_wrote = 0;

  color::rgba color_tmp = color;

  //calculate rgb spektrum for spezific alpha value, because
  //led board is not supporting alpha values
  color.r *= color.a;
  color.g *= color.a;
  color.b *= color.a;
  //led board value spektrum is from 0 - 999.
  //at r@w's led strip, 0 means fully lighted and 999 light off(_invertMask)
  color.r = (fabs(_invertMask-color.r) * 999.0);
  color.g = (fabs(_invertMask-color.g) * 999.0);
  color.b = (fabs(_invertMask-color.b) * 999.0);

  _ssOut.clear();
  _ssOut.str("");
  _ssOut << (int)color.r << " " << (int)color.g << " " << (int)color.b << "\n\r";

  // send data over serial port
  bytes_wrote = _serialIO->sendData(_ssOut.str());
  if(bytes_wrote == -1)
  {
    ROS_WARN("Can not write to serial port. Port closed!");
  }
  else
  {
    ROS_DEBUG("Wrote [%s] with %i bytes from %i bytes", \
              _ssOut.str().c_str(), bytes_wrote, (int)_ssOut.str().length());
    m_sigColorSet(color_tmp);
  }
}
