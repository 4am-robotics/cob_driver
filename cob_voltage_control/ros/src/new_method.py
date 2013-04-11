#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_relayboard')
import rospy
import time
import csv
from cob_relayboard.msg import EmergencyStopState
from std_msgs.msg import Float64
import savitzky
import numpy as np
from math import *
from pr2_msgs.msg import PowerState

class volts_filter():
    
    def __init__(self):
        self.volts = 0.
        self.wsize = 61
        self.filter_order = 3
        self.theta = rospy.get_param("/volts_filt/theta")
        self.off_y = rospy.get_param("/volts_filt/off_y")
        self.abcd = rospy.get_param("/volts_filt/abcd")
        self.sg = savitzky.savitzky_golay(window_size=self.wsize, order=self.filter_order)
        size = 2*self.wsize+1
        self.volt_filt = 48000*np.ones(size)
        
        rospy.Subscriber("/power_board/voltage", Float64, self.callback)
        
        self.pub_power = rospy.Publisher('/power_state', PowerState)
        self.msg_power = PowerState()
        
    def callback(self, data):
    
        self.volts = data.data
        
        if(data.data <= 44000):
            self.volts = 44000
            time_r = 0.
        elif(data.data >= 48000):
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

        rospy.loginfo(self.t_est/3600)
        
        self.msg_power.header.stamp = rospy.Time.now()
        self.msg_power.time_remaining.secs = self.t_est
        self.msg_power.prediction_method = 'thiago_test'
        self.msg_power.relative_capacity = (self.t_est/13500) * 100
        
        rospy.loginfo(self.volt_filt)
        
        self.pub_power.publish(self.msg_power)
            
if __name__ == '__main__':
    rospy.init_node('volt_filt')
    vf = volts_filter()

    while not rospy.is_shutdown():
    
        
        rospy.sleep(1.0)



