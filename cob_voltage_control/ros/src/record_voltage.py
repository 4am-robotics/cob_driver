#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_relayboard')
import rospy
import time
import csv
from cob_relayboard.msg import EmergencyStopState
from std_msgs.msg import Float64
#from pr2_msgs.msg import PowerState

#starttime = 1

def callback(data):
	writer.writerow( ( round((rospy.Time.now() - starttime).to_sec(),5), data.data) )

#def timer_callback(data):
#	rospy.loginfo(data.data)

def record():
	rospy.init_node('record_voltage')
	global starttime
	starttime = rospy.Time.now()

	global f
	global writer
	filename = rospy.get_param("/record_voltage/filename")
	f = open(filename, 'wt', 1)
	writer = csv.writer(f)

	rospy.Subscriber("/power_board/voltage", Float64, callback)
	

	while not rospy.is_shutdown():
		#pub_em_stop.publish(msg_em)
		#pub_power.publish(msg_power) comes already out of gazebo
		rospy.sleep(1.0)

if __name__ == '__main__':
	record()

