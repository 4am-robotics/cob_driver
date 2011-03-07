#!/usr/bin/env python

PKG='cob_undercarriage_ctrl'
import roslib; roslib.load_manifest(PKG)

import sys
import unittest

import rospy

from cob_srvs.srv import *

class InitTest(unittest.TestCase):
    def __init__(self, *args):
		super(InitTest, self).__init__(*args)
		rospy.init_node("test_init_node")

    def test_init(self):
		# call init service
		rospy.wait_for_service('init',10)
		call_init = rospy.ServiceProxy('init', Trigger)
		try:
			resp1 = call_init()
		except rospy.ServiceException, e:
			self.assertTrue(False, "calling init service failed: " +str(e))

		# evaluate response
		self.assertTrue(resp1.success.data, "init not succesfull: "+ resp1.error_message.data)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_init', InitTest)
