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
from cob_light.msg import LightMode

def changeColor():
	pub = rospy.Publisher('light_controller/command_mode', LightMode, queue_size=1)
	rospy.init_node('light_test')
	light_mode = LightMode()
	#color in rgb color space ranging from 0 to 1
	red = ColorRGBA()
	red.r = 1
	red.g = 0
	red.b = 0
	red.a = 1

	yellow = ColorRGBA()
	yellow.r = 0.4
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

	for color in [red,yellow,green,white,blue,green]:
		rospy.loginfo("Setting rgb to %s [%d, %d, %d]",color.r,color.g,color.b,color.a)
		light_mode.colors= 27*[color]
		pub.publish(light_mode)
		time.sleep(3)

if __name__ == '__main__':
    try:
        changeColor()
    except rospy.ROSInterruptException: pass

