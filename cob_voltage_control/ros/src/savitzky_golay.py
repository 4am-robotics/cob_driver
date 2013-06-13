#!/usr/bin/env python

#################################################################
##\file
#
# \note
# Copyright (c) 2013 \n
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA) \n\n
#
#################################################################
#
# \note
# Project name: Care-O-bot Research
# \note
# ROS package name:
#
# \author
# Author: Thiago de Freitas Oliveira Araujo,
# email:thiago.de.freitas.oliveira.araujo@ipa.fhg.de
# \author
# Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: April 2013
#
# \brief
# Battery Characterization for the IPA robots
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the Fraunhofer Institute for Manufacturing
# Engineering and Automation (IPA) nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see < http://www.gnu.org/licenses/>.
#
#################################################################
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
        
        self.msg_power.header.stamp = rospy.Time.now()
        self.msg_power.time_remaining.secs = self.t_est
        self.msg_power.prediction_method = '3rd_order_polynom'
        self.msg_power.relative_capacity = (self.t_est/11748) * 100
        
        self.pub_power.publish(self.msg_power)
            
if __name__ == '__main__':
    rospy.init_node('volt_filt')
    vf = volts_filter()

    while not rospy.is_shutdown():
    
        
        rospy.sleep(1.0)



