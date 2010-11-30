#!/usr/bin/env python
PACKAGE='cob_sdh'
import roslib; roslib.load_manifest(PACKAGE)
import sys
import rospy
import unittest
import time
from cob_msgs.msg import *
from cob_srvs.srv import *
from pr2_controllers_msgs.msg import *
from sensor_msgs.msg import *

class TestInititialization(unittest.TestCase):
    def __init__(self, *args):
		super(TestInititialization, self).__init__(*args)
		rospy.init_node("test_init")

    def callback(self, msg):
		if(msg.name[0] == "sdh_thumb_2_joint"):
			self.received = True
    def callback_tac(self, msg):
		self.received_tac = True

    def test_init(self):
		self.received = False
		self.received_tac = False
		rospy.wait_for_service('init')
		call_init = rospy.ServiceProxy('init', Trigger)
		try:
			resp1 = call_init(0)
		except rospy.ServiceException, e:
			self.assertEquals(0, 1, "Init failed: " +str(e))
		sub = rospy.Subscriber("/joint_states", JointState, self.callback)
		sub_tac = rospy.Subscriber("tactile_data", TactileSensor, self.callback_tac)
		time.sleep(1.0)
		if(self.received):
			self.assertEquals(1, 1, "Init Hand successful")
		else:
			self.assertEquals(0, 1, "No controller state messages received")
		if(self.received_tac):
			self.assertEquals(1, 1, "Init Tactile successful")
		else:
			self.assertEquals(0, 1, "No tactile messages received")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PACKAGE, 'test_init', TestInititialization)
