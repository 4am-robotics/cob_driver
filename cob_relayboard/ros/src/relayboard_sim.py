#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_relayboard')
import rospy
import time
from cob_relayboard.msg import EmergencyStopState
#from pr2_msgs.msg import PowerState

def relayboard_sim():
	rospy.init_node('cob_relayboard_sim')

	# emergency_stop topic
	pub_em_stop = rospy.Publisher('/emergency_stop_state', EmergencyStopState)
	msg_em = EmergencyStopState()
	msg_em.emergency_button_stop = False
	msg_em.scanner_stop = False
	msg_em.emergency_state = 0

	# power_state topic
	#pub_power = rospy.Publisher('/power_state', PowerState)
	#msg_power = PowerState()
	#msg_power.header.stamp = rospy.Time.now()
	#msg_power.time_remaining.secs = 1000
	#msg_power.relative_capacity = 70

	while not rospy.is_shutdown():
		pub_em_stop.publish(msg_em)
		#pub_power.publish(msg_power) comes already out of gazebo
		rospy.sleep(1.0)

if __name__ == '__main__':
	try:
		relayboard_sim()
	except rospy.ROSInterruptException: 
		print "Interupted"
		pass
