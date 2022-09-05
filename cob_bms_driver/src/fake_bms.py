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
from cob_srvs.srv import SetInt, SetIntResponse
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from diagnostic_updater import Updater

class FakeBMS(object):
    def __init__(self):
        self._srv_current              = rospy.Service('~set_current', SetFloat, self.current_cb)
        self._srv_relative_remaining_capacity   = rospy.Service('~set_relative_remaining_capacity', SetInt, self.relative_remaining_capacity_cb)
        self._poll_frequency           = rospy.get_param('~poll_frequency_hz', 20.0)
        self._pub_voltage              = rospy.Publisher('~voltage', Float64, queue_size = 1)
        self._pub_current              = rospy.Publisher('~current', Float64, queue_size = 1)
        self._pub_remaining_capacity   = rospy.Publisher('~remaining_capacity', Float64, queue_size = 1)
        self._pub_full_charge_capacity = rospy.Publisher('~full_charge_capacity', Float64, queue_size = 1)
        self._pub_temparature          = rospy.Publisher('~temperature', Float64, queue_size = 1)

        self._updater = Updater()
        self._updater.setHardwareID("bms")
        self._updater.add("cob_bms_dagnostics_updater", self.produce_diagnostics)

        self._voltage              = rospy.get_param('~voltage', 48.0) # V
        self._current              = rospy.get_param('~current', -8.0) # A
        self._remaining_capacity   = rospy.get_param('~remaining_capacity', 35.0) # Ah
        self._full_charge_capacity = rospy.get_param('~full_charge_capacity', 35.0) # Ah
        self._temperature          = rospy.get_param('~temperature', 25.0) # °C

        rospy.Timer(rospy.Duration(1.0), self.publish_diagnostics)
        rospy.Timer(rospy.Duration(1.0/self._poll_frequency), self.timer_cb)
        rospy.Timer(rospy.Duration(1.0/self._poll_frequency), self.timer_consume_power_cb)

    def current_cb(self, req):
        self._current = round(req.data,2)
        res_current = SetFloatResponse(True, "Set current to {}".format(self._current))
        return res_current

    def relative_remaining_capacity_cb(self, req):
        self._remaining_capacity = round(((req.data * self._full_charge_capacity)/100.0), 3)
        res_capacity = SetIntResponse(True, "Set remaining capacity to {}".format(self._remaining_capacity))
        return res_capacity

    def publish_diagnostics(self, event):
        self._updater.update()

    def produce_diagnostics(self, stat):
        stat.summary(DiagnosticStatus.OK, "Fake Driver: Ready")
        stat.add("current[A]", self._current)
        stat.add("voltage[V]", self._voltage)
        stat.add("temperature[Celsius]", self._temperature)
        stat.add("remaining_capacity[Ah]", round(self._remaining_capacity, 3))
        stat.add("full_charge_capacity[Ah]", self._full_charge_capacity)
        return stat

    def timer_cb(self, event):
        self._pub_voltage.publish(self._voltage)
        self._pub_current.publish(self._current)
        self._pub_remaining_capacity.publish(round(self._remaining_capacity, 3))
        self._pub_full_charge_capacity.publish(self._full_charge_capacity)
        self._pub_temparature.publish(self._temperature)

    def timer_consume_power_cb(self, event):
        # emulate the battery usage based on the current values
        self._remaining_capacity += (self._current/self._poll_frequency)/3600.0
        if self._remaining_capacity <= 0.0:
            self._remaining_capacity = 0.0
        if self._remaining_capacity >= self._full_charge_capacity:
            self._remaining_capacity = round(self._full_charge_capacity,3)

if __name__ == '__main__':
    rospy.init_node('fake_bms')
    FakeBMS()
    rospy.spin()
