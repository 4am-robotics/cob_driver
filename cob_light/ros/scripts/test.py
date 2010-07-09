#!/usr/bin/python
#***************************************************************
#
# Copyright (c) 2010
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
# Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# Date of creation: June 2010
# ToDo:
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
# but WITHOUT ANY WARRANTY; withouself.ser = serial.Serial('/dev/ttyUSB0', 230400)t even the implied warranty of
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
from cob_msgs.msg import Light

def changeColor():
	pub = rospy.Publisher('light_controller/command', Light)
	rospy.init_node('light_test')
	#color in rgb color space ranging from 0 to 999
	red = Light()
	red.r = 999
	red.g = 0
	red.b = 0
	
	yellow = Light()
	yellow.r = 400
	yellow.g = 999
	yellow.b = 0
	
	green = Light()
	green.r = 0
	green.g = 999
	green.b = 0
	
	blue = Light()
	blue.r = 0
	blue.g = 0
	blue.b = 999
	
	white = Light()
	white.r = 300
	white.g = 999
	white.b = 300
	
	for color in [red,yellow,green,blue,white]:
		rospy.loginfo("Setting rgb to [%d, %d, %d]",color.r,color.g,color.b)
		pub.publish(color)
		time.sleep(3)

if __name__ == '__main__':
    try:
        changeColor()
    except rospy.ROSInterruptException: pass

