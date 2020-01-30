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


import numpy as np
import rospy
from cob_msgs.msg import PowerState
from cob_phidgets.msg import AnalogSensor


class PowerStatePhidget():
    PHIDGET_MAX_VALUE = 999
    PHIDGET_MIN_VALUE = 0
    PERIOD_RECORD_SIZE = 6
    VOLTAGE_COLLECTION_TIME = 6.0  # sec

    def __init__(self):
        self.voltage = None
        self.current = None
        self.last_update = rospy.Time(0)
        self.charging = False
        try:
            self.voltage_divider_factor = rospy.get_param("~voltage_divider_factor")
        except KeyError:
            raise KeyError("Parameter \"~voltage_divider_factor\" not found on parameter server.")
        self.voltage_full = rospy.get_param("~voltage_full", 52.0)
        self.voltage_empty = rospy.get_param("~voltage_empty", 38.0)
        self.current_max = rospy.get_param("~current_max", 30.0)
        self.current_min = rospy.get_param("~current_min", -30.0)

        self.pub_power_state = rospy.Publisher('power_state', PowerState, queue_size=1)
        self.sub_analog_sensors = rospy.Subscriber("analog_sensors", AnalogSensor, self.phidget_cb)

        self.pr_next = 0
        self.period_record = []
        self.cb_avg_time = 0.1
        self.voltage_bag_maxlen = 100
        self.voltage_bag = []

    def append_voltage_bag(self, num):
        while len(self.voltage_bag) >= self.voltage_bag_maxlen:
            self.voltage_bag.pop(0)
        self.voltage_bag.append(num)

    def calculate_voltage(self):
        if len(self.voltage_bag) > 0:
            self.voltage = np.mean(self.voltage_bag)

    def phidget_cb(self, msg):
        # Estimate commands frequency; we do continuously as it can be very different depending on the
        # publisher type, and we don't want to impose extra constraints to keep this package flexible
        if len(self.period_record) < self.PERIOD_RECORD_SIZE:
            self.period_record.append((rospy.Time.now() - self.last_update).to_sec())
        else:
            self.period_record[self.pr_next] = (rospy.Time.now() - self.last_update).to_sec()

        self.pr_next += 1
        self.pr_next %= len(self.period_record)
        self.last_update = rospy.Time.now()

        if len(self.period_record) <= self.PERIOD_RECORD_SIZE / 2:
            # wait until we have some values; make a reasonable assumption (10 Hz) meanwhile
            self.cb_avg_time = 0.1
        else:
            # enough; recalculate with the latest input
            self.cb_avg_time = np.median(self.period_record)

        # now set the max voltage bag size
        self.voltage_bag_maxlen = int(self.VOLTAGE_COLLECTION_TIME / self.cb_avg_time)

        voltage_raw = None
        current_raw = None

        for i in range(0, len(msg.uri)):
            if msg.uri[i] == "voltage":
                voltage_raw = msg.value[i]
            if msg.uri[i] == "current":
                current_raw = msg.value[i]

        if voltage_raw != None:
            # Calculation of real voltage
            voltage = self.voltage_divider_factor * voltage_raw / self.PHIDGET_MAX_VALUE
            voltage = round(voltage, 3)
            self.append_voltage_bag(voltage)

        if current_raw != None:
            # Calculation of real current
            self.current = self.current_min + (self.current_max - self.current_min) * (current_raw -
                                                                                       self.PHIDGET_MIN_VALUE) / (self.PHIDGET_MAX_VALUE - self.PHIDGET_MIN_VALUE)
            self.current = round(self.current, 3)

            if self.current > 0:
                self.charging = True
            else:
                self.charging = False

    def calculate_power_consumption(self):
        if not self.charging and self.voltage != None and self.current != None:
            return round(self.voltage * abs(self.current), 3)
        else:
            return 0.0

    def calculate_relative_remaining_capacity(self):
        percentage = None
        if self.voltage != None:
            percentage = round((self.voltage - self.voltage_empty) * 100 / (self.voltage_full - self.voltage_empty), 3)
            percentage = min(percentage, 100)
            percentage = max(percentage, 0)
            return percentage
        else:
            return 0.0

    def publish(self):
        self.calculate_voltage()
        if self.voltage != None and self.current != None and (rospy.Time.now() - self.last_update) < rospy.Duration(1):
            ps = PowerState()
            ps.header.stamp = self.last_update
            ps.voltage = self.voltage
            ps.current = self.current
            ps.power_consumption = self.calculate_power_consumption()
            ps.relative_remaining_capacity = self.calculate_relative_remaining_capacity()
            ps.charging = self.charging
            self.pub_power_state.publish(ps)

if __name__ == "__main__":
    rospy.init_node("power_state_phidget")
    try:
        psp = PowerStatePhidget()
    except KeyError as e:
        rospy.logerr("Shutting down: {}".format(e))
        exit(1)

    rospy.loginfo("power state phidget running")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        psp.publish()
        rate.sleep()
