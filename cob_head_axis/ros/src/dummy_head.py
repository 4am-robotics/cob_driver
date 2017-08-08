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
from sensor_msgs.msg import JointState

def talker():
    pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
    rospy.init_node('dummy_head')
    while not rospy.is_shutdown():
        msg = JointState()
        msg.name = []
        msg.name.append('head_axis_joint')
        msg.position = []
        msg.position.append(0)
#        msg.position.append(-3.14)
        pub.publish(msg)
        rospy.sleep(0.1)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

