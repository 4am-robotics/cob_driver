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
import numpy
import warnings
from std_msgs.msg import Float64
from cob_msgs.msg import PowerState
from sensor_msgs.msg import BatteryState

class PowerStateAggregator():

    def __init__(self):
        # get parameters
        self.current_buffer_size = rospy.get_param('~current_buffer_size', 901) # approx. 60sec, topic rate 15Hz, use odd number to prevent 0.0 mean
        self.pub_power_state = rospy.Publisher('power_state', PowerState, queue_size=1)
        self.pub_battery_state = rospy.Publisher('battery_state', BatteryState, queue_size=1)
        self.voltage = None
        self.current = None
        self.last_currents = []
        self.last_update = rospy.Time(0)
        self.connected = False
        self.charging = False
        self.remaining_capacity = None
        self.full_charge_capacity = None
        self.temperature = None
        rospy.Subscriber("voltage", Float64, self.voltage_cb)
        rospy.Subscriber("current", Float64, self.current_cb)
        rospy.Subscriber("remaining_capacity", Float64, self.remaining_capacity_cb)
        rospy.Subscriber("full_charge_capacity", Float64, self.full_charge_capacity_cb)
        rospy.Subscriber("temperature", Float64, self.temperature_cb)

    def voltage_cb(self, msg):
        self.last_update = rospy.Time.now()
        self.voltage = msg.data

    def current_cb(self, msg):
        self.last_update = rospy.Time.now()
        self.current = msg.data

        # fill current into list of past currents for filtering purposes
        if len(self.last_currents) >= self.current_buffer_size:
            self.last_currents.pop(0)
        self.last_currents.append(msg.data)

        if msg.data > 0:
            self.charging = True
        else:
            self.charging = False
        
        if msg.data > -1: # we use a limit of -1Ampere because if the battery is 100% full and the robot is still docked, there is no more current going into the battery. -1 A is biggger than the "Ruhestrom", so this should be ok until BMS is fixed and delivers a proper flag for docked or not_docked.
            self.connected = True
        else:
            self.connected = False
        
    def remaining_capacity_cb(self, msg):
        self.last_update = rospy.Time.now()
        self.remaining_capacity = msg.data

    def full_charge_capacity_cb(self, msg):
        self.last_update = rospy.Time.now()
        self.full_charge_capacity = msg.data

    def temperature_cb(self, msg):
        self.last_update = rospy.Time.now()
        self.temperature = msg.data

    def calculate_power_consumption(self):
        if not self.charging and self.voltage != None and self.current != None:
            return round(self.voltage * abs(self.current), 3)
        else:
            return 0.0

    def calculate_relative_remaining_capacity(self):
        try:
            return round(100.0*(self.remaining_capacity/self.full_charge_capacity), 3)
        except ZeroDivisionError as e:
            rospy.logerr("ZeroDivisionError: full_charge_capacity is 0.0: %s" % (e))
        except:
            rospy.logwarn("something went wrong, cannot calculate relative remaining capacity. full_charge_capacity=%s, remaining_capacity=%s" % (self.full_charge_capacity, self.remaining_capacity))
        return 0.0

    def calculate_time_remaining(self):
        warnings.filterwarnings('error')
        time_remaining_max = 10.0 # assume 10h - for cases where current approx. 0A
        time_remaining = time_remaining_max
        try:
            if len(self.last_currents) > 0:
                current = numpy.mean(self.last_currents)

                if self.full_charge_capacity != None and self.remaining_capacity != None:
                    if self.charging:
                        time_remaining = round((self.full_charge_capacity - self.remaining_capacity) / abs(current), 3)
                    else:
                        time_remaining = round(self.remaining_capacity / abs(current), 3)
                else:
                    pass
            else:
                pass
        except ZeroDivisionError as e:
            rospy.logerr("ZeroDivisionError: current is 0.0: {}".format(e))
        except Warning as w:
            rospy.logerr("Warning: {}".format(w))
        except Exception as e:
            rospy.logerr("Exception: {}".format(e))
        # rospy.logdebug("calculate_time_remaining")
        # rospy.logdebug("time_remaining_max: {}".format(time_remaining_max))
        # rospy.logdebug("time_remaining: {}".format(time_remaining))
        # rospy.logdebug("self.last_currents: {}".format(self.last_currents))
        # rospy.logdebug("current: {}".format(current))
        # rospy.logdebug("self.full_charge_capacity: {}".format(self.full_charge_capacity))
        # rospy.logdebug("self.remaining_capacity: {}".format(self.remaining_capacity))
        # rospy.logdebug("self.charging: {}".format(self.charging))
        return min(time_remaining, time_remaining_max)
 
    def publish(self):
        if self.voltage != None and self.current != None and self.remaining_capacity != None and self.full_charge_capacity != None and self.temperature != None and (rospy.Time.now() - self.last_update) < rospy.Duration(1):
            power_consumption = self.calculate_power_consumption()
            relative_remaining_capacity = self.calculate_relative_remaining_capacity()
            time_remaining = self.calculate_time_remaining()

            ps = PowerState()
            ps.header.stamp = self.last_update
            ps.voltage = self.voltage
            ps.current = self.current
            ps.power_consumption = power_consumption
            ps.remaining_capacity = self.remaining_capacity
            ps.relative_remaining_capacity = relative_remaining_capacity
            ps.connected = self.connected
            ps.charging = self.charging
            ps.time_remaining = time_remaining
            ps.temperature = self.temperature

            bs = BatteryState()
            bs.header.stamp = self.last_update
            bs.voltage = self.voltage
            bs.current = self.current
            bs.charge = self.remaining_capacity
            bs.design_capacity = self.full_charge_capacity
            bs.percentage = relative_remaining_capacity / 100.0
            bs.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.charging else BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
            bs.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            bs.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
            bs.present = True
            bs.cell_voltage = [self.voltage]
            bs.location = "emulated_battery"
            bs.serial_number = "emulated_battery"

            self.pub_power_state.publish(ps)
            self.pub_battery_state.publish(bs)

if __name__ == "__main__":
    rospy.init_node("power_state_aggregator")
    PSA = PowerStateAggregator()
    rospy.loginfo("power state aggregator running")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        PSA.publish()
        try:
            rate.sleep()
        except rospy.exceptions.ROSInterruptException as e:
            pass
        
