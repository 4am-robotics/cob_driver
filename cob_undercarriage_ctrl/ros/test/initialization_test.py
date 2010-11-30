#!/usr/bin/env python
PACKAGE='cob_undercarriage_ctrl'
import roslib; roslib.load_manifest(PACKAGE)
import sys
import rospy
import unittest
import time
from nav_msgs.msg import *
from cob_srvs.srv import *
from pr2_controllers_msgs.msg import *

class TestInititialization(unittest.TestCase):
    def __init__(self, *args):
		super(TestInititialization, self).__init__(*args)
		rospy.init_node("test_init")

    def callback(self, msg):
		self.received = True

    def test_init(self):
		self.received = False
		rospy.wait_for_service('init')
		call_init = rospy.ServiceProxy('init', Trigger)
		try:
			resp1 = call_init(0)
		except rospy.ServiceException, e:
			self.assertEquals(0, 1, "Init failed: " +str(e))
		sub = rospy.Subscriber("odometry", Odometry, self.callback)
		time.sleep(1.0)
		if(self.received):
			self.assertEquals(1, 1, "Init successful")
		else:
			self.assertEquals(0, 1, "No controller state messages received")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PACKAGE, 'test_init', TestInititialization)
