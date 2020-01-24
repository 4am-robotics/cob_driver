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


import roslib
import rospy
import sys

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the set_mimic action, including the
# goal message and the result message.
import cob_mimic.msg

def mimic_client():
    # Creates the SimpleActionClient, passing the type of the action
    # to the constructor.
    client = actionlib.SimpleActionClient('/mimic/set_mimic', cob_mimic.msg.SetMimicAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = cob_mimic.msg.SetMimicGoal()
    goal.mimic = sys.argv[1]

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        if len(sys.argv) != 2:
            print("Error: please specify mimic string as input")
            sys.exit(1)

        rospy.init_node('mimic_test_node')
        result = mimic_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
