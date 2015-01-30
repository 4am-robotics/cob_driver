#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_relayboard')
import rospy
import time
from cob_msgs.msg import EmergencyStopState
from cob_msgs.msg import PowerBoardState
from std_msgs.msg import Float64
from std_srvs.srv import Empty

class RelayboardSim:

	# emergency_stop_state topic
	pub_em_stop = rospy.Publisher('/emergency_stop_state', EmergencyStopState, queue_size=1)
	msg_em = EmergencyStopState()

	# power_board/state topic
	pub_power_board = rospy.Publisher('/power_board/state', PowerBoardState, queue_size=1)
	msg_power_board = PowerBoardState()

	# power_board/voltage topic
	pub_voltage = rospy.Publisher('/power_board/voltage', Float64, queue_size=1)
	msg_voltage = Float64()

	# constructor
	def __init__(self):

		# initial values for simulation

		self.msg_em.emergency_button_stop = False
		self.msg_em.scanner_stop = False
		self.msg_em.emergency_state = 0 # system is operating normal

		self.msg_power_board.header.stamp = rospy.Time.now()
		self.msg_power_board.run_stop = True
		self.msg_power_board.wireless_stop = True # for cob the wireless stop field is misused as laser stop field

		self.msg_voltage.data = 48.0 # in simulation battery is always full

		# advertise services
		srv = rospy.Service('~toggle_emergency_button_stop', Empty, self.toggle_emergency_button_stop)
		srv = rospy.Service('~toggle_scanner_stop', Empty, self.toggle_scanner_stop)
		
	# service callback to simulate emergency button stop
	def toggle_emergency_button_stop(self, req):
		
		self.msg_em.emergency_button_stop = not self.msg_em.emergency_button_stop
		return ()

	# service callback to simulate laser scanner stop
	def toggle_scanner_stop(self, req):	

		self.msg_em.scanner_stop = not self.msg_em.scanner_stop
		return ()		

def relayboard_sim():

	# start up node
	rospy.init_node('cob_relayboard_sim')
	relayboard_sim = RelayboardSim()

	while not rospy.is_shutdown():

		# initial situation: system operating normal
		if relayboard_sim.msg_em.emergency_state == 0:

			if relayboard_sim.msg_em.emergency_button_stop == 1 or relayboard_sim.msg_em.scanner_stop == 1:
				relayboard_sim.msg_em.emergency_state = 1
				rospy.logerr("Emergency stop issued.")

		# initial situation: emergency stop was issued
		if relayboard_sim.msg_em.emergency_state == 1:

			if relayboard_sim.msg_em.emergency_button_stop == 0 and relayboard_sim.msg_em.scanner_stop == 0:
				relayboard_sim.msg_em.emergency_state = 2 # emergency stop was confirmed
				rospy.loginfo("Emergency stop confirmed.")

		if relayboard_sim.msg_em.emergency_state == 2:

			if relayboard_sim.msg_em.emergency_button_stop == 1 or relayboard_sim.msg_em.scanner_stop == 1:
				relayboard_sim.msg_em.emergency_state = 1
				rospy.logerr("Emergency stop issued.")

			else:
				relayboard_sim.msg_em.emergency_state = 0
				rospy.loginfo("Emergency stop released.")
	
		# publish topics
		relayboard_sim.pub_em_stop.publish(relayboard_sim.msg_em)
		relayboard_sim.pub_power_board.publish(relayboard_sim.msg_power_board)
		relayboard_sim.pub_voltage.publish(relayboard_sim.msg_voltage)

		rospy.sleep(0.5)

if __name__ == '__main__':
	
	try:
		relayboard_sim()

	except rospy.ROSInterruptException:
		print "Interupted"
		pass
