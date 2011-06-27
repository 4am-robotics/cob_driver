#! /usr/bin/env python

import roslib
roslib.load_manifest('cob_sound')
import rospy

import os

import actionlib

from std_msgs.msg import String
from cob_sound.msg import *
from cob_sound.srv import *

class SoundAction(object):

	def __init__(self):
		self._as = actionlib.SimpleActionServer("say", SayAction, execute_cb=self.as_cb)
		rospy.Subscriber("/say", String, self.topic_cb)
		rospy.Service('/say', SayText, self.service_cb)
		self._as.start()

	def as_cb(self, goal):
		print "begin"
		self.say(goal.text.data)
		self._as.set_succeeded()
		print "end"
		
	def topic_cb(self,msg):
		self.say(msg.data)

	def service_cb(self,msg):
		self.say(msg.text)
		res = SayTextResponse()
		return res

	def say(self,text):
		rospy.loginfo('Saying: %s' % (text))
		os.system("echo " + text + " | text2wave | aplay -q &")		

if __name__ == '__main__':
	rospy.init_node('cob_sound')
	SoundAction()
	rospy.loginfo("cob_sound is running")
	rospy.spin()

