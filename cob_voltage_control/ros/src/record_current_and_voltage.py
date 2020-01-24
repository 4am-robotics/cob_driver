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


import csv

import rospy
from cob_phidgets.msg import AnalogSensor

def callback(data):
	#print "voltage=", data.value[1] # voltage sensor is on the second port of the phidgets board
	#print "current=", data.value[0] # current sensor is on the first port of the phidgets board
	writer.writerow( ( round((rospy.Time.now() - starttime).to_sec(),5), data.value[1], data.value[0]) )

def record():
	rospy.init_node('record_current_and_voltage')
	global starttime
	starttime = rospy.Time.now()

	global f
	global writer
	filename = rospy.get_param("~filename")
	global port_voltage, port_current
	port_voltage = rospy.get_param("~port_voltage")
	port_current = rospy.get_param("~port_current")
	f = open(filename, 'wt', 1)
	writer = csv.writer(f)
	writer.writerow(("time_from_start", "voltage", "current"))

	rospy.Subscriber("/analog_sensors", AnalogSensor, callback)

	rospy.spin()

if __name__ == '__main__':
	record()

