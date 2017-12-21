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
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from cob_srvs.srv import SetFloat, SetFloatResponse
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from diagnostic_updater import Updater

class FakeBMS(object):
    def __init__(self):
        self.srv_current              = rospy.Service('~set_current', SetFloat, self.current_cb)
        self.srv_relative_remaining_capacity   = rospy.Service('~set_relative_remaining_capacity', SetFloat, self.relative_remaining_capacity_cb)
        self.poll_frequency           = rospy.get_param('~poll_frequency_hz', 20.0)
        self.pub_voltage              = rospy.Publisher('~voltage', Float64, queue_size = 1)
        self.pub_current              = rospy.Publisher('~current', Float64, queue_size = 1)
        self.pub_remaining_capacity   = rospy.Publisher('~remaining_capacity', Float64, queue_size = 1)
        self.pub_full_charge_capacity = rospy.Publisher('~full_charge_capacity', Float64, queue_size = 1)
        self.pub_temparature          = rospy.Publisher('~temperature', Float64, queue_size = 1)

        self.updater = Updater()
        self.updater.setHardwareID("bms")
        self.updater.add("cob_bms_dagnostics_updater", self.produce_diagnostics)

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
        
    def relative_remaining_capacity_cb(self, req):
        self.remaining_capacity = round(((req.data * self.full_charge_capacity)/100.0), 3)
        res_capacity = SetFloatResponse(True, "Set relative remaining capacity to {}".format(req.data))
        return res_capacity

    def publish_diagnostics(self, event):
        self.updater.update()

    def produce_diagnostics(self, stat):
        stat.summary(DiagnosticStatus.OK, "Fake Driver: Ready")
        stat.add("current[A]", self.current)
        stat.add("voltage[V]", self.voltage)
        stat.add("temperature[Celsius]", self.temperature)
        stat.add("remaining_capacity[Ah]", self.remaining_capacity)
        stat.add("full_charge_capacity[Ah]", self.full_charge_capacity)
        return stat

    def timer_cb(self, event):
        self.voltage = 48.0
        self.pub_voltage.publish(self.voltage)
        self.pub_current.publish(self.current)
        self.pub_remaining_capacity.publish(self.remaining_capacity)
        self.pub_full_charge_capacity.publish(self.full_charge_capacity)
        self.pub_temparature.publish(self.temperature)

    def timer_consume_power_cb(self, event):
        # emulate the battery usage based on the current values
        self.remaining_capacity += (self.current/self.poll_frequency)/3600.0
        if self.remaining_capacity <= 0.0:
            self.remaining_capacity = 0.0
        if self.remaining_capacity >= self.full_charge_capacity:
            self.remaining_capacity = self.full_charge_capacity
        
if __name__ == '__main__':
  rospy.init_node('fake_bms')
  FakeBMS()
  rospy.spin()
