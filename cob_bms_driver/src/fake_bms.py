#!/usr/bin/env python

import rospy
from cob_msgs.msg import PowerState
from cob_srvs.srv import SetFloat, SetFloatResponse
from std_srvs.srv import SetBool, SetBoolResponse

class FakeBMS(object):
    def __init__(self):
        self.srv_charging = rospy.Service('~set_charging', SetBool, self.charging_cb)
        self.srv_relative_remaining_capacity = rospy.Service('~set_relative_remaining_capacity', SetFloat, self.relative_remaining_capacity_cb)
        self.pub_power_state = rospy.Publisher('/power_state', PowerState, queue_size = 1)
        rospy.Timer(rospy.Duration(1), self.timer_cb)
        self.power_state = PowerState()
        rospy.loginfo('FakeBMS is running')

    def charging_cb(self, req):
        self.power_state.charging = req.data
        res_charging = SetBoolResponse(True, "Set charging to {}".format(req.data))
        return res_charging
        
    def relative_remaining_capacity_cb(self, req):
        self.power_state.relative_remaining_capacity = req.data
        res_capacity = SetFloatResponse(True, "Set relative remaining capacity to {}".format(req.data))
        return res_capacity

    def timer_cb(self, event):
        self.power_state.voltage = 0.0
        self.power_state.current = 0.0
        self.power_state.power_consumption = 0.0
        self.power_state.remaining_capacity = 0.0
        self.power_state.time_remaining = 0.0
        self.power_state.temperature = 0.0
        self.pub_power_state.publish(self.power_state)

if __name__ == '__main__':
  rospy.init_node('fake_bms')
  FakeBMS()
  rospy.spin()
