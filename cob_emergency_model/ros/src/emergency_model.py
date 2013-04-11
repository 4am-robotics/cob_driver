#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_emergency_model')
import rospy
import time
import csv
from cob_relayboard.msg import EmergencyStopState
from geometry_msgs.msg import PolygonStamped, Point32, PointStamped
from sensor_msgs.msg import LaserScan
import math
from math import *


class em_stop():
    
    def __init__(self):
        self.topic = 'em_stop_polygon'
        
        self.pol_pub = rospy.Publisher(self.topic, PolygonStamped)    
        self.pol_msg = PolygonStamped()
        
        self.points = rospy.get_param("points")
        
        self.laser_msg = None
        
        self.angle_min = 0.
        self.angle_increment = 0.
        
        self.laser_ranges = None
        
        rospy.Subscriber("/scan_front", LaserScan, self.callback)
        
        self.x = 0.
        self.y = 0.
        
  # adapted from: http://www.ariel.com.au/a/python-point-int-poly.html
     
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
        
        for i in range(len(self.laser_ranges)):
            if (self.laser_ranges[i] == 0):
                pass
            
            elif(self.laser_ranges[i] <= 0.12):
                angle = self.angle_min + self.angle_increment*i
                self.x = self.laser_ranges[i] * cos(self.angle)
                self.y = self.laser_ranges[i] * sin(self.angle)
        
        
        em_trig = self.point_in_poly(self.x,self.y,self.points)
        rospy.loginfo(em_trig)
        
        if(em_trig):
            rospy.loginfo("Laser Emergency Stop Issued")
            
        rospy.loginfo(self.angle_min)
            
if __name__ == '__main__':

    rospy.init_node('em_stop')
    em = em_stop()

    while not rospy.is_shutdown():

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
