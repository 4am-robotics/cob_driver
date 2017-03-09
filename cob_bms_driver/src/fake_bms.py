#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from cob_srvs.srv import SetFloat, SetFloatResponse
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class FakeBMS(object):
    def __init__(self):
        self.srv_current              = rospy.Service('~set_current', SetFloat, self.current_cb)
        self.srv_remaining_capacity   = rospy.Service('~set_remaining_capacity', SetFloat, self.remaining_capacity_cb)
        self.poll_frequency           = rospy.get_param('~poll_frequency_hz', 20.0)
        self.pub_voltage              = rospy.Publisher('~voltage', Float64, queue_size = 1)
        self.pub_current              = rospy.Publisher('~current', Float64, queue_size = 1)
        self.pub_remaining_capacity   = rospy.Publisher('~remaining_capacity', Float64, queue_size = 1)
        self.pub_full_charge_capacity = rospy.Publisher('~full_charge_capacity', Float64, queue_size = 1)
        self.pub_temparature          = rospy.Publisher('~temperature', Float64, queue_size = 1)
        self.pub_diagnostics          = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

        self.voltage              = 0.0
        self.current              = -8.0
        self.remaining_capacity   = 35.0
        self.full_charge_capacity = 35.0 # Ah
        self.temperature          = 0.0

        rospy.Timer(rospy.Duration(1.0), self.publish_diagnostics)
        rospy.Timer(rospy.Duration(1.0/self.poll_frequency), self.timer_cb)
        rospy.Timer(rospy.Duration(1.0/self.poll_frequency), self.timer_consume_power_cb)

    def current_cb(self, req):
        self.current = req.data
        res_current = SetFloatResponse(True, "Set current to {}".format(req.data))
        return res_current
        
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
        self.voltage = 48.0
        self.pub_voltage.publish(self.voltage)
        self.pub_current.publish(self.current)
        self.pub_remaining_capacity.publish(self.remaining_capacity)
        self.pub_full_charge_capacity.publish(self.full_charge_capacity)
        self.pub_temparature.publish(self.temperature)

    def timer_consume_power_cb(self, event):
        # emulate the battery usage based on the current values
        self.remaining_capacity += (self.current/self.poll_frequency)/3600   
        if self.remaining_capacity <= 0:
            self.remaining_capacity = 0
        if self.remaining_capacity >= self.full_charge_capacity:
            self.remaining_capacity = self.full_charge_capacity
        
if __name__ == '__main__':
  rospy.init_node('fake_bms')
  FakeBMS()
  rospy.spin()
