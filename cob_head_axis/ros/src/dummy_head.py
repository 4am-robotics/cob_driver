#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_head_axis')
import rospy
from sensor_msgs.msg import JointState
def talker():
    pub = rospy.Publisher('/joint_states', JointState)
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

