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

#include <colorOMarker.h>
#include <std_msgs/ColorRGBA.h>
#include <cob_light/ColorRGBAArray.h>
#include <geometry_msgs/Pose.h>

ColorOMarker::ColorOMarker(ros::NodeHandle* nh)
{
  p_nh = nh;
  _pubMarkerArray = p_nh->advertise<visualization_msgs::MarkerArray>("markers", 1);
}

ColorOMarker::~ColorOMarker()
{
}

bool ColorOMarker::init()
{
  return true;
}

void ColorOMarker::setColor(color::rgba color)
{
  visualization_msgs::MarkerArray marker_array;

  for (auto i = 0; i < this->getNumLeds(); ++i)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "light";
    marker.header.stamp = ros::Time::now();
    marker.ns = std::string("led");
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = geometry_msgs::Pose();
    marker.pose.position.x = 0.01*i;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = color.a;
    marker_array.markers.push_back(marker);
  }
  _pubMarkerArray.publish(marker_array);
}

void ColorOMarker::setColorMulti(std::vector<color::rgba> &colors)
{
  visualization_msgs::MarkerArray marker_array;

  for (auto i = 0; i < colors.size(); ++i)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "light";
    marker.header.stamp = ros::Time::now();
    marker.ns = std::string("led");
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = geometry_msgs::Pose();
    marker.pose.position.x = 0.01*i;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = colors[i].r;
    marker.color.g = colors[i].g;
    marker.color.b = colors[i].b;
    marker.color.a = colors[i].a;
    marker_array.markers.push_back(marker);
  }
  _pubMarkerArray.publish(marker_array);

  //_pubSimulationMulti.publish(colorMsg);
  m_sigColorSet(colors[0]);
}
