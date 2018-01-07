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

#include <colorOSim.h>
#include <std_msgs/ColorRGBA.h>
#include <cob_light/ColorRGBAArray.h>

ColorOSim::ColorOSim(ros::NodeHandle* nh)
{
  p_nh = nh;
  _pubSimulation = p_nh->advertise<std_msgs::ColorRGBA>("debug",2);
  _pubSimulationMulti = p_nh->advertise<cob_light::ColorRGBAArray>("debugMulti", 2);
}

ColorOSim::~ColorOSim()
{
}

bool ColorOSim::init()
{
  return true;
}

void ColorOSim::setColor(color::rgba color)
{
  std_msgs::ColorRGBA _color;
  _color.r = color.r;
  _color.g = color.g;
  _color.b = color.b;
  _color.a = color.a;

  _pubSimulation.publish(_color);
  m_sigColorSet(color);
}

void ColorOSim::setColorMulti(std::vector<color::rgba> &colors)
{
  std_msgs::ColorRGBA color;
  cob_light::ColorRGBAArray colorMsg;
  for(size_t i = 0; i < colors.size(); ++i)
  {
    color.r = colors[i].r;
    color.g = colors[i].g;
    color.b = colors[i].b;
    color.a = colors[i].a;
    colorMsg.colors.push_back(color);
  }

  _pubSimulationMulti.publish(colorMsg);
  m_sigColorSet(colors[0]);
}
