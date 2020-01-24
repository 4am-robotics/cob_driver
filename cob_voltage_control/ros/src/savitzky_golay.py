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


from math import cos, sin
import numpy as np

import rospy
import savitzky
from std_msgs.msg import Float64
from cob_msgs.msg import EmergencyStopState, PowerState

class volts_filter():

    def __init__(self):
        self.volts = 0.
        self.wsize = 61
        self.filter_order = 3
        self.theta = rospy.get_param("~theta")
        self.off_y = rospy.get_param("~off_y")
        self.abcd = rospy.get_param("~abcd")
        self.maximum_time = rospy.get_param("~maximum_time")
        self.sg = savitzky.savitzky_golay(window_size=self.wsize, order=self.filter_order)
        size = 2*self.wsize+1
        self.volt_filt = 48000*np.ones(size)

        rospy.Subscriber("/power_board/voltage", Float64, self.callback)

        self.pub_power = rospy.Publisher('/power_state', PowerState, queue_size=1)
        self.msg_power = PowerState()

    def callback(self, data):

        self.volts = data.data
        self.volts = self.volts*1000

        if(self.volts <= 44000):
            self.volts = 44000
        elif(self.volts >= 48000):
            self.volts = 48000

        self.process_voltage()

    def process_voltage(self):

        self.volt_filt = np.insert(self.volt_filt, 0, self.volts)
        self.volt_filt = np.delete(self.volt_filt, -1)

        vfilt = self.sg.filter(self.volt_filt)

        old_settings = np.seterr(all='raise')

        self.t_est = np.polyval(self.abcd, vfilt[self.wsize])

        self.t_est = vfilt[self.wsize]*sin(self.theta) + self.t_est*cos(self.theta)
        self.t_est = self.t_est + self.off_y

        if(self.t_est <0):
            self.t_est = 0

        self.msg_power.header.stamp = rospy.Time.now()
        self.msg_power.time_remaining.secs = self.t_est
        #self.msg_power.prediction_method = '3rd_order_polynom'
        #self.msg_power.relative_capacity = (self.t_est/self.maximum_time) * 100
        #self.msg_power.AC_present = 0

        self.pub_power.publish(self.msg_power)

if __name__ == '__main__':
    rospy.init_node('volt_filt')
    vf = volts_filter()

    while not rospy.is_shutdown():
        rospy.sleep(1.0)
