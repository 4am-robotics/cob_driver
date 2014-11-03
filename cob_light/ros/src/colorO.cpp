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

#include <colorO.h>
#include <ros/ros.h>

ColorO::ColorO(SerialIO* serialIO)
{
  _serialIO = serialIO;
}

ColorO::~ColorO()
{
}

void ColorO::setColorMulti(std::vector<color::rgba> &colors, std::vector<int> &led_numbers)
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
