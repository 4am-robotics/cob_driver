#!/usr/bin/python

import time
from random import *

import roslib
roslib.load_manifest('cob_camera_sensors')
roslib.load_manifest('cob_script_server')
import rospy

import tf
from tf.transformations import euler_from_quaternion

import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from math import *


from simple_script_server import script

class CalibCam(script):

	def Initialize(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/stereo/left/image_color",Image,self.callback)
		self.cv_image = cv.CreateImage((1,1), 1 , 3)
		self.sss.init("torso")
		self.sss.init("sdh")

	def Run(self):
		print "start"
		seed()
		maxVal = 0.1
		file_path = "/home/goa/cal/"
		listener = tf.TransformListener()
		nr_images = 14

		# move components to initial position
		self.sss.move("arm","calib")
		self.sss.move("torso","home")
		self.sss.move("sdh","home")

		self.sss.wait_for_input()
		self.sss.move("sdh","calib")
		self.sss.wait_for_input()

		# start calbration routine
		for i in range(1,nr_images):
			if i==1:
				r1 = maxVal
				r2 = 2*maxVal
			elif i==2:
				r1 = -maxVal
				r2 = 2*maxVal
			elif i==3:
				r1 = maxVal
				r2 = -2*maxVal
			elif i==4:
				r1 = -maxVal
				r2 = -2*maxVal
			else:	
				r1 = (random()-0.5)*2*maxVal;
				r2 = 2*(random()-0.5)*2*maxVal;
			self.sss.move("torso",[[0.75*r1,r2,r1]])
			self.sss.sleep(1)
			try:
				listener.waitForTransform('/base_link', '/head_color_camera_l_link',rospy.Time(0),rospy.Duration(1.0))
				(trans,rot) = listener.lookupTransform('/base_link', '/head_color_camera_l_link', rospy.Time(0))
				rpy = euler_from_quaternion(rot)
				cyaw = cos(rpy[2])
				syaw = sin(rpy[2])
				cpitch = cos(rpy[1])
				spitch = sin(rpy[1])
				croll = cos(rpy[0])
				sroll = sin(rpy[0])
				R11 = cyaw*cpitch
				R12 = cyaw*spitch*sroll-syaw*croll
				R13 = cyaw*spitch*croll+syaw*sroll
				R21 = syaw*cpitch
				R22 = syaw*spitch*sroll+cyaw*croll
				R23 = syaw*spitch*croll-cyaw*sroll
				R31 = -spitch
				R32 = cpitch*sroll
				R33 = cpitch*croll
				fout = open(file_path+'calpic'+str(i)+'.coords','w')
				fout.write(str(R11)+' '+str(R12)+' '+str(R13)+' '+str(trans[0]*1000)+'\n'+str(R21)+' '+str(R22)+' '+str(R23)+' '+str(trans[1]*1000)+'\n'+str(R31)+' '+str(R32)+' '+str(R33)+' '+str(trans[2]*1000))
				fout.close()
			except (tf.LookupException, tf.ConnectivityException):
				print "tf exception"

			self.sss.sleep(1)
			cv.SaveImage(file_path+'calpic'+str(i)+'.png',self.cv_image)
			self.sss.sleep(1)
		self.sss.move("torso","home")
		print "finished"
		
	def callback(self,data):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
		except CvBridgeError, e:
			print e
		
if __name__ == "__main__":
	SCRIPT = CalibCam()
	SCRIPT.Start()
