#! /usr/bin/env python

import rospy
import actionlib
from cob_sound.msg import *

def say_client():
	client = actionlib.SimpleActionClient('say', SayAction)
	client.wait_for_server()

	# Creates a goal to send to the action server.
	goal = SayGoal()
	goal.text = "Hello, how are you? I am fine."

	# Sends the goal to the action server.
	client.send_goal(goal)

	# Waits for the server to finish performing the action.
	client.wait_for_result()

	# Prints out the result of executing the action
	return client.get_result()
	rospy.loginfo("Say action finished")

if __name__ == '__main__':
	try:
		rospy.init_node('say_client')
		result = say_client()
	except rospy.ROSInterruptException:
		print "program interrupted before completion"

