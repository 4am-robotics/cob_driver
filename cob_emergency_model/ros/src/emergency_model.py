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
# Modeling the laser safety region for IPA robots on simulations
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

import roslib; roslib.load_manifest('cob_emergency_model')
import rospy
import time
import csv
from cob_relayboard.msg import EmergencyStopState
from geometry_msgs.msg import PolygonStamped, Point32, PointStamped
from sensor_msgs.msg import LaserScan
import math
from math import *

#################################
# Emergency Stop Class
################################

class em_stop():
    def __init__(self):
        self.topic = 'em_stop_polygon'
        
        self.pol_pub = rospy.Publisher(self.topic, PolygonStamped)    
        self.pol_msg = PolygonStamped()
        
        self.points = rospy.get_param("/em_model/points")
        
        self.laser_msg = None
        
        self.checking = False
        
        self.angle_min = 0.
        self.angle_increment = 0.
        
        self.laser_ranges = None
        
        # Subscriber for the laser front scanner
        rospy.Subscriber("/scan_front", LaserScan, self.callback)
        
        self.x = 0.
        self.y = 0.
        
        # Publisher definitions for the emergency stop
        self.pub_em_stop = rospy.Publisher('/emergency_stop_state', EmergencyStopState)
        self.msg_em = EmergencyStopState()
        self.msg_em.emergency_button_stop = False
        self.msg_em.scanner_stop = False
        self.msg_em.emergency_state = 0
    
    #################################
    # This functions checks if the point is inside the polygon
    # using the Ray-Casting method
    # adapted from: http://www.ariel.com.au/a/python-point-int-poly.html
    ################################
         
    def point_in_poly(self, x,y,poly):

        n = len(poly)
        self.inside = False

        p1x = poly[0][0]
        p1y = poly[0][1]

        for i in range(n+1):
            p2x = poly[i % n][0]
            p2y = poly[i % n][1]
            if y > min(p1y,p2y):
                if y <= max(p1y,p2y):
                    if x <= max(p1x,p2x):
                        if p1y != p2y:
                            xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or x <= xinters:
                            self.inside = not self.inside
            p1x,p1y = p2x,p2y

        return self.inside

    
    def callback(self, msg):

        self.laser_msg = msg
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        
        self.laser_ranges = msg.ranges
        
        if(not self.checking):
            self.checking = True
            
            for i in range(len(self.laser_ranges)):
                if (self.laser_ranges[i] == 0): # discarding zeroed range values
                    pass
                
                elif(self.laser_ranges[i] < 0.1196):
                    # converting the radius/angle polar coordinates to x-y
                    self.angle = self.angle_min + self.angle_increment*i
                    self.x = self.laser_ranges[i] * cos(self.angle)
                    self.y = self.laser_ranges[i] * sin(self.angle)
            
                    # checks if the transformed point is inside the polygon
                    em_trig = self.point_in_poly(self.x,self.y,self.points)
                    rospy.loginfo(em_trig)
            
                    if(em_trig):
                        # publishing the Laser Emergency stop message
                        self.msg_em.emergency_button_stop = False
                        self.msg_em.scanner_stop = True
                        self.msg_em.emergency_state = 1
                        self.pub_em_stop.publish(self.msg_em)
                        rospy.loginfo("Laser Emergency Stop Issued")
                        return
            
                # otherwise publish message with no emergency stop
                self.msg_em.emergency_button_stop = False
                self.msg_em.scanner_stop = False
                self.msg_em.emergency_state = 0
                self.pub_em_stop.publish(self.msg_em)

            self.checking = False
            
if __name__ == '__main__':

    rospy.init_node('em_stop')
    em = em_stop()

    while not rospy.is_shutdown():

        # this publishes the polygonal regional marker for checking on rviz
        em.pol_msg.header.frame_id = "/base_laser_front_link"
        em.pol_msg.header.stamp = rospy.Time.now()


        for el in em.points:
            point = Point32()

            point.x = el[0]
            point.y = el[1]
            point.z = 0
            em.pol_msg.polygon.points.append( point )

        em.pol_pub.publish( em.pol_msg )
        

        rospy.sleep(1.0)
