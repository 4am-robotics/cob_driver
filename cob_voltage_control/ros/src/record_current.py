#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_voltage_control')
import rospy
import time
import csv
from std_msgs.msg import Float64
from cob_phidgets.msg import *
#from pr2_msgs.msg import PowerState

#starttime = 1

def callback(data):
	#print data.value[0] # current sensor is on the first port of the phidgets board 
	writer.writerow( ( round((rospy.Time.now() - starttime).to_sec(),5), data.value[0]) )

#def timer_callback(data):
#	rospy.loginfo(data.data)

def record():
	rospy.init_node('record_current')
	global starttime
	starttime = rospy.Time.now()

	global f
	global writer
	filename = rospy.get_param("/record_current/filename")
	f = open(filename, 'wt', 1)
	writer = csv.writer(f)

	rospy.Subscriber("/analog_sensors", AnalogSensor, callback)
	

	while not rospy.is_shutdown():
		rospy.sleep(1.0)

if __name__ == '__main__':
	record()

