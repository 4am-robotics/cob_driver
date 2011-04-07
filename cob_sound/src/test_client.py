#! /usr/bin/env python

import roslib
roslib.load_manifest('cob_sound')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from cob_sound.msg import *

def say_client():
	# Creates the SimpleActionClient, passing the type of the action
	# (FibonacciAction) to the constructor.
	client = actionlib.SimpleActionClient('sound_controller/say', SayAction)

	# Waits until the action server has started up and started
	# listening for goals.
	client.wait_for_server()

	# Creates a goal to send to the action server.
	goal = SayGoal()
	goal.text.data = "Hello, how are you? I am fine."

	# Sends the goal to the action server.
	client.send_goal(goal)

	# Waits for the server to finish performing the action.
	client.wait_for_result()

	# Prints out the result of executing the action
	return client.get_result()  # A FibonacciResult
	rospy.loginfo("Say action finished")

if __name__ == '__main__':
	try:
		# Initializes a rospy node so that the SimpleActionClient can
		# publish and subscribe over ROS.
		rospy.init_node('say_client')
		result = say_client()
	except rospy.ROSInterruptException:
		print "program interrupted before completion"

