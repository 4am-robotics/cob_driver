#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_relayboard')
import rospy
import time
from cob_relayboard.msg import EmergencyStopState
from pr2_msgs.msg import PowerBoardState
from std_msgs.msg import Float64

def relayboard_sim():
	rospy.init_node('cob_relayboard_sim')

	# emergency_stop topic
	pub_em_stop = rospy.Publisher('/emergency_stop_state', EmergencyStopState)
	msg_em = EmergencyStopState()
	msg_em.emergency_button_stop = False
	msg_em.scanner_stop = False
	msg_em.emergency_state = 0

	# power_board/state topic
	pub_power_board = rospy.Publisher('/power_board/state', PowerBoardState)
	msg_power_board = PowerBoardState()
	msg_power_board.header.stamp = rospy.Time.now()
	msg_power_board.run_stop = True
	msg_power_board.wireless_stop = True #for cob the wireless stop field is misused as laser stop field

	# power_board/voltage topic
	pub_voltage = rospy.Publisher('/power_board/voltage', Float64)
	msg_voltage = Float64()
	msg_voltage.data = 48.0 # in simulation battery is always full

	while not rospy.is_shutdown():
		pub_em_stop.publish(msg_em)
		pub_power_board.publish(msg_power_board)
		pub_voltage.publish(msg_voltage)
		rospy.sleep(1.0)

if __name__ == '__main__':
	try:
		relayboard_sim()
	except rospy.ROSInterruptException:
		print "Interupted"
		pass
