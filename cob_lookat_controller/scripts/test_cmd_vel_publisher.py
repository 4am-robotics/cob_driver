#!/usr/bin/env python
import roslib;
roslib.load_manifest('cob_lookat_controller')
import rospy
from brics_actuator.msg import JointVelocities, JointValue

def start_node():
    rospy.init_node('test_cmd_vel_publisher', anonymous=True)
    #advertise topic
    pub = rospy.Publisher('/lookat_controller/command_vel', JointVelocities)
    rospy.sleep(1.0)
    
    msg = JointVelocities()
    value_lin = JointValue()
    value_lin.timeStamp = rospy.Time.now()
    value_lin.joint_uri = 'lookat_lin_joint'
    value_lin.unit = 'rad'
    value_lin.value = 0.1
    msg.velocities.append(value_lin)
    value_x = JointValue()
    value_x.timeStamp = rospy.Time.now()
    value_x.joint_uri = 'lookat_x_joint'
    value_x.unit = 'rad'
    value_x.value = 0.0
    msg.velocities.append(value_x)
    value_y = JointValue()
    value_y.timeStamp = rospy.Time.now()
    value_y.joint_uri = 'lookat_y_joint'
    value_y.unit = 'rad'
    value_y.value = 0.0
    msg.velocities.append(value_y)
    value_z = JointValue()
    value_z.timeStamp = rospy.Time.now()
    value_z.joint_uri = 'lookat_z_joint'
    value_z.unit = 'rad'
    value_z.value = 0.0
    msg.velocities.append(value_z)

    #print msg
    rospy.loginfo("publishing now!")
    pub.publish(msg)
    rospy.sleep(1.0)


if __name__ == '__main__':
    start_node()

