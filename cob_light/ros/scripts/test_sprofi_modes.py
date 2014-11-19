#!/usr/bin/python
#***************************************************************
#
# Copyright (c) 2014
#
# Fraunhofer Institute for Manufacturing Engineering	
# and Automation (IPA)
#
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#
# Project name: care-o-bot
# ROS stack name: cob_driver
# ROS package name: cob_light
#								
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#			
# Author: Thiago de Freitas, email:tdf@ipa.fhg.de
# Supervised by: Thiago de Freitas, email:tdf@ipa.fhg.de
#
# Date of creation: October 2014
#
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fraunhofer Institute for Manufacturing 
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#****************************************************************

import time
import roslib
roslib.load_manifest('cob_light')
import rospy
from std_msgs.msg import ColorRGBA
from cob_light.srv import *
from cob_light.msg import *

def changeColor():
  rospy.wait_for_service('/light_controller/mode')
  control_lights = rospy.ServiceProxy('/light_controller/mode', SetLightMode)
  
  light_mode = LightMode()
  #color in rgb color space ranging from 0 to 1
  red = ColorRGBA()
  red.r = 1
  red.g = 0
  red.b = 0
  red.a = 1

  yellow = ColorRGBA()
  yellow.r = 1
  yellow.g = 1
  yellow.b = 0
  yellow.a = 1

  green = ColorRGBA()
  green.r = 0
  green.g = 1
  green.b = 0
  green.a = 1

  blue = ColorRGBA()
  blue.r = 0
  blue.g = 0
  blue.b = 1
  blue.a = 1

  white = ColorRGBA()
  white.r = 0.3
  white.g = 1
  white.b = 0.3
  white.a = 1

  for i in range(1,6):
    for color in [red,yellow,green,white,blue,green]:
      rospy.loginfo("Setting rgb to %s [%d, %d, %d]",color.r,color.g,color.b,color.a)
      light_mode.mode = i
      light_mode.frequency=10
      light_mode.color = color
      try:
        resp1 = control_lights(light_mode)
        print resp1
      except rospy.ServiceException, e:
        print "Service call failed: %s"%e
      
      time.sleep(6)

if __name__ == '__main__':
    try:
        rospy.init_node('light_test')
        changeColor()
    except rospy.ROSInterruptException: 
        pass

