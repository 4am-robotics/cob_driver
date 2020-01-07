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
		print("program interrupted before completion")

