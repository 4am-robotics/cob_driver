#!/usr/bin/env python

import rospy
from cob_phidgets.msg import AnalogSensor
from cob_msgs.msg import PowerState

class PowerStatePhidget():
    PHIDGET_MAX_VALUE = 999
    PHIDGET_MIN_VALUE = 0

    def __init__(self):
        self.voltage = None
        self.current = None
        self.last_update = rospy.Time(0)
        self.charging = False

        self.voltage_max = rospy.get_param("~voltage_max", 59.5)
        self.voltage_full = rospy.get_param("~voltage_full", 52.0)
        self.voltage_empty = rospy.get_param("~voltage_empty", 38.0)
        self.current_max = rospy.get_param("~current_max", 30.0)
        self.current_min = rospy.get_param("~current_min", -30.0)

        self.pub_power_state = rospy.Publisher('power_state', PowerState, queue_size=1)
        self.sub_analog_sensors = rospy.Subscriber("analog_sensors", AnalogSensor, self.phidget_cb)

    def phidget_cb(self, msg):
        self.last_update = rospy.Time.now()
        voltage_raw = None
        current_raw = None

        for i in range(0, len(msg.uri)):
            if msg.uri[i] == "voltage":
                voltage_raw = msg.value[i]
            if msg.uri[i] == "current":
                current_raw = msg.value[i]

        if voltage_raw != None:
            #Calculation of real voltage
            self.voltage = voltage_raw * self.voltage_max/self.PHIDGET_MAX_VALUE;
            self.voltage = round(self.voltage, 3)

        if current_raw != None:
            #Calculation of real current
            self.current = self.current_min+(self.current_max - self.current_min)*(current_raw - self.PHIDGET_MIN_VALUE) / (self.PHIDGET_MAX_VALUE - self.PHIDGET_MIN_VALUE)
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
            percentage = round((self.voltage - self.voltage_empty) * 100/(self.voltage_full - self.voltage_empty), 3)
            percentage = min(percentage, 100)
            percentage = max(percentage, 0)
            return percentage
        else:
            return 0.0

    def publish(self):
        if self.voltage != None and self.current != None and (rospy.Time.now() - self.last_update) < rospy.Duration(1):
            ps = PowerState()
            ps.header.stamp = self.last_update
            ps.voltage = self.voltage
            ps.current = self.current
            ps.power_consumption = self.calculate_power_consumption()
            ps.relative_remaining_capacity = self.calculate_relative_remaining_capacity()
            ps.charging = self.charging
            self.pub_power_state.publish(ps)
            return ps

if __name__ == "__main__":
    rospy.init_node("power_state_phidget")
    psp = PowerStatePhidget()
    rospy.loginfo("power state phidget running")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        psp.publish()
        rate.sleep()
