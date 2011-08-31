#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_relayboard')
import rospy
import time
from cob_relayboard.msg import EmergencyStopState
def talker():
    pub = rospy.Publisher('/emergency_stop_state', EmergencyStopState)
    rospy.init_node('cob_relayboard_sim')
    em = EmergencyStopState()
    em.emergency_button_stop = False
    em.scanner_stop = False
    em.emergency_state = 0
    while not rospy.is_shutdown():
        pub.publish(em)
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: 
	print "Interupted"
	pass
