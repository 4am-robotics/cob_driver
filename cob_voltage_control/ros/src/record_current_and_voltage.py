#!/usr/bin/env python

import rospy
from cob_phidgets.msg import *

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

