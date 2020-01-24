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
from std_msgs.msg import Float64
from cob_msgs.msg import EmergencyStopState

def relayboard_sim():
	rospy.init_node('cob_relayboard_sim')

	# emergency_stop topic
	pub_em_stop = rospy.Publisher('emergency_stop_state', EmergencyStopState, queue_size=1)
	msg_em = EmergencyStopState()
	msg_em.emergency_button_stop = False
	msg_em.scanner_stop = False
	msg_em.emergency_state = 0

	# voltage topic
	pub_voltage = rospy.Publisher('voltage', Float64, queue_size=1)
	msg_voltage = Float64()
	msg_voltage.data = 48.0 # in simulation battery is always full

	while not rospy.is_shutdown():
		pub_em_stop.publish(msg_em)
		pub_voltage.publish(msg_voltage)
		rospy.sleep(1.0)

if __name__ == '__main__':
	try:
		relayboard_sim()
	except rospy.ROSInterruptException:
		print("Interupted")
		pass
