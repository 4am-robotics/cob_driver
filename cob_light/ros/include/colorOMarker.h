/*
 * Copyright 2021 Mojin Robotics GmbH https://www.mojin-robotics.de
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


#ifndef COLOROMARKER_H
#define COLOROMARKER_H

#include <iColorO.h>

#include <colorUtils.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class ColorOMarker : public IColorO
{
public:
  ColorOMarker(ros::NodeHandle* nh);
  virtual ~ColorOMarker();

  bool init();
  void setColor(color::rgba color);
  void setColorMulti(std::vector<color::rgba> &colors);

private:
  ros::NodeHandle* p_nh;
  ros::Publisher _pubMarkerArray;
};

#endif
