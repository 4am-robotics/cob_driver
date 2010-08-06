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
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#****************************************************************

import roslib; 
roslib.load_manifest('cob_light')
import rospy
from cob_msgs.msg import Light

import serial
import sys

class LightControl:
	def __init__(self):
		self.ns_global_prefix = "/light_controller"
		
		# get parameter from parameter server
		if not rospy.has_param(self.ns_global_prefix + "/devicestring"):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/devicestring")
			sys.exit()
		devicestring_param = rospy.get_param(self.ns_global_prefix + "/devicestring")
		if not rospy.has_param(self.ns_global_prefix + "/baudrate"):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/baudrate")
			sys.exit()
		baudrate_param = rospy.get_param(self.ns_global_prefix + "/baudrate")
		
		# open serial communication
		rospy.loginfo("trying to initializing serial connection")
		try:
			self.ser = serial.Serial(devicestring_param, baudrate_param)
		except serial.serialutil.SerialException:
			rospy.logerr("Could not initialize serial connection, aborting...")
			sys.exit()
		rospy.loginfo("serial connection initialized successfully")

	def setRGB(self, red, green, blue):
		#color in rgb color space ranging from 0 to 999
		#print "setRGB", red, green, blue
		if(red <= 999 and green <= 999 and blue <= 999):
			self.ser.write(str(red)+ " " + str(green)+ " " + str(blue)+"\n\r")

	def LightCallback(self,light):
		rospy.loginfo("Received new color: rgb = [%d, %d, %d]",light.r,light.g,light.b)
		print light.name.data
		self.setRGB(light.r,light.g,light.b)

if __name__ == '__main__':
	rospy.init_node('light_controller')
	lc = LightControl()
	rospy.Subscriber("command", Light, lc.LightCallback)
	rospy.loginfo(rospy.get_name() + " running")
	rospy.spin()
