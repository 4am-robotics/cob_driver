#!/usr/bin/env python

PKG='cob_powercube_chain'
import roslib; roslib.load_manifest(PKG)

import sys
import unittest

import rospy

class APITest(unittest.TestCase):
	def __init__(self, *args):
		super(APITest, self).__init__(*args)
		rospy.init_node("test_api_node")
		self.namespace = rospy.get_namespace()
		
	def test_service_api(self):
		rospy.wait_for_service('init',10)
		rospy.wait_for_service('stop',10)
		rospy.wait_for_service('recover',10)
		rospy.wait_for_service('set_operation_mode',10)

	def test_topic_api(self):
		topic_list = rospy.get_published_topics()
		#print topic_list
		
		# generate check list
		topic_check_list = []
		topic_check_list.append(['/joint_states','sensor_msgs/JointState'])
		topic_check_list.append([self.namespace + 'state','pr2_controllers_msgs/JointTrajectoryControllerState'])
		topic_check_list.append([self.namespace + 'joint_trajectory_action/feedback','pr2_controllers_msgs/JointTrajectoryActionFeedback'])
		topic_check_list.append([self.namespace + 'joint_trajectory_action/result','pr2_controllers_msgs/JointTrajectoryActionResult'])
		topic_check_list.append([self.namespace + 'joint_trajectory_action/status','actionlib_msgs/GoalStatusArray'])
		
		# perform check
		for L in topic_check_list:
			if not L in topic_list:
				self.assertTrue(False, L[0] + ' not in topic_list')

if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG, 'api_test', APITest)
