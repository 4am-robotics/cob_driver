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


import rospy
from sensor_msgs.msg import *

class GazeboVirtualRangeSensor():

	def __init__(self, laser_topic, range_topic):
		rospy.Subscriber(laser_topic, LaserScan, self.laser_callback)
#		rospy.logdebug("subscribed to laser topic: %s", laser_topic)
		self.pub = rospy.Publisher(range_topic, Range, queue_size=1)
#		rospy.loginfo("topic '" + range_topic + "' advertised")
		rospy.loginfo("adding sensor converting from topic '" + laser_topic + "' to topic '" + range_topic + "'")

	def laser_callback(self, msg):
		sensor_range = Range()

		# set header
		sensor_range.header = msg.header

		# set range from average out of laser message
		if len(msg.ranges) > 0:
			sensor_range.range = sum(msg.ranges)/len(msg.ranges)
		else:
			sensor_range.range = 0

		# set min and max range
		sensor_range.min_range = msg.range_min
		sensor_range.max_range = msg.range_max

		# set field of view
		sensor_range.field_of_view = msg.angle_max - msg.angle_min

		# set radiation type to IR (=1)
		sensor_range.radiation_type = 1

		# publish range
		self.pub.publish(sensor_range)

if __name__ == "__main__":
	rospy.init_node('tactile_sensors')
	rospy.sleep(0.5)

	if not rospy.has_param('~sensors'):
		rospy.logerr("no sensors specified")
		exit(1)

	sensors = rospy.get_param('~sensors')

	if type(sensors) != list:
		rospy.logerr("sensors parameter not a list")
		exit(1)

	for sensor in sensors:
		GazeboVirtualRangeSensor(sensor['laser_topic'], sensor['range_topic'])

	rospy.spin()
