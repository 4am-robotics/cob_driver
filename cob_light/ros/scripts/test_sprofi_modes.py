#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import time

import rospy
from std_msgs.msg import ColorRGBA
from cob_light.srv import *
from cob_light.msg import *

def changeColor():
  rospy.wait_for_service('/light_controller/mode')
  control_lights = rospy.ServiceProxy('/light_controller/mode', SetLightMode)
  rospy.init_node('light_test')
  light_mode = LightMode()
  #color in rgb color space ranging from 0 to 1
  red = ColorRGBA()
  red.r = 1
  red.g = 0
  red.b = 0
  red.a = 0.7

  light_red = ColorRGBA()
  light_red.r = 1
  light_red.g = 0
  light_red.b = 0
  light_red.a = 0.3

  off = ColorRGBA()
  off.r = 1
  off.g = 0
  off.b = 0
  off.a = 0.01

  dimm = ColorRGBA()
  dimm.r = 1
  dimm.g = 0
  dimm.b = 0
  dimm.a = 0.04

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
  blue.g = 1
  blue.b = 0.7
  blue.a = 0.4

  white = ColorRGBA()
  white.r = 0.3
  white.g = 1
  white.b = 0.3
  white.a = 1

  light_mode.mode = 6
  light_mode.frequency=100
  seq1 = Sequence()
  seq1.color = blue
  seq1.hold_time = 4;
  seq1.cross_time = 1;
  light_mode.sequences.append(seq1);

  seq2 = Sequence()
  seq2.color = red;
  seq2.hold_time = 4;
  seq2.cross_time = 1;
  light_mode.sequences.append(seq2);

 # seq3 = Sequence()
 # seq3.color = off;
 # seq3.hold_time = 0.06;
 # seq3.cross_time = 0.05;
 # light_mode.sequences.append(seq3);

 # seq4 = Sequence()
 # seq4.color = light_red;
 # seq4.hold_time = 0.08;
 # seq4.cross_time = 0.05;
 # light_mode.sequences.append(seq4);
 #
  try:
    resp1 = control_lights(light_mode)
    print(resp1)
  except rospy.ServiceException as e:
    print("Service call failed: %s"%e)

  time.sleep(6)

if __name__ == '__main__':
    try:
        rospy.init_node('light_test')
        changeColor()
    except rospy.ROSInterruptException:
        pass

