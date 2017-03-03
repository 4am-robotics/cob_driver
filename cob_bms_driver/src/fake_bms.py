#!/usr/bin/env python

import rospy
from random import uniform
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from cob_srvs.srv import SetFloat, SetFloatResponse
from std_srvs.srv import SetBool, SetBoolResponse
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class FakeBMS(object):
    def __init__(self):
        self.srv_charging             = rospy.Service('~set_charging', SetBool, self.charging_cb)
        self.srv_remaining_capacity   = rospy.Service('~set_remaining_capacity', SetFloat, self.remaining_capacity_cb)
        self.poll_frequency           = rospy.get_param('~poll_frequency_hz', 20.0)
        self.pub_charging_state       = rospy.Publisher('~battery_charging', Bool, queue_size = 1)
        self.pub_voltage              = rospy.Publisher('~voltage', Float64, queue_size = 1)
        self.pub_current              = rospy.Publisher('~current', Float64, queue_size = 1)
        self.pub_remaining_capacity   = rospy.Publisher('~remaining_capacity', Float64, queue_size = 1)
        self.pub_full_charge_capacity = rospy.Publisher('~full_charge_capacity', Float64, queue_size = 1)
        self.pub_temparature          = rospy.Publisher('~temperature', Float64, queue_size = 1)
        self.pub_diagnostics          = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

        self.charging_state       = False
        self.voltage              = 0.0
        self.current              = 0.0
        self.remaining_capacity   = 0.0
        self.full_charge_capacity = 1.0
        self.temperature          = 0.0
        self.current_polarity     = 1.0 if self.charging_state else -1.0

        rospy.Timer(rospy.Duration(1.0), self.publish_diagnostics)
        rospy.Timer(rospy.Duration(1.0/self.poll_frequency), self.timer_cb)

    def charging_cb(self, req):
        self.charging_state    = req.data
        self.current_polarity  = 1.0 if self.charging_state else -1.0
        res_charging = SetBoolResponse(True, "Set charging to {}".format(req.data))
        return res_charging
        
    def remaining_capacity_cb(self, req):
        self.remaining_capacity = req.data
        res_capacity = SetFloatResponse(True, "Set remaining capacity to {}".format(req.data))
        return res_capacity


    def publish_diagnostics(self, event):
        msg = DiagnosticArray()
        msg.header.stamp = rospy.get_rostime()

        status = DiagnosticStatus()
        status.name = rospy.get_name()
        status.level = DiagnosticStatus.OK
        status.message = "fake diagnostics"
        status.hardware_id = rospy.get_name()
        msg.status.append(status)

        self.pub_diagnostics.publish(msg)

    def timer_cb(self, event):
        self.voltage = 50.0 + uniform(1,5) # ground voltage plus fluctuation
        self.current = self.current_polarity*(4.0 + uniform(1,2))
        self.pub_charging_state.publish(self.charging_state)
        self.pub_voltage.publish(self.voltage)
        self.pub_current.publish(self.current)
        self.pub_remaining_capacity.publish(self.remaining_capacity)
        self.pub_full_charge_capacity.publish(self.full_charge_capacity)
        self.pub_temparature.publish(self.temperature)
        
if __name__ == '__main__':
  rospy.init_node('fake_bms')
  FakeBMS()
  rospy.spin()
