#!/usr/bin/python
#################################################################
##\file
#
# \note
# Copyright (c) 2010 \n
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA) \n\n
#
#################################################################
#
# \note
# Project name: Care-O-bot Research
# \note
# ROS stack name: cob_environments
# \note
# ROS package name: cob_gazebo_objects
#
# \author
# Author: Nadia Hammoudeh Garcia, email:nadia.hammoudeh.garcia@ipa.fhg.de
# \author
# Supervised by: Nadia Hammoudeh Garcia, email:nadia.hammoudeh.garcia@ipa.fhg.de
#
# \date Date of creation: Nov 2014
#
# \brief
# Implements script server functionalities.
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

import sys
import roslib
roslib.load_manifest('cob_mimic')

import os
import subprocess

import rospy
import Image
import actionlib

from cob_mimic.srv import *
from cob_mimic.msg import *

class Mimic:
  def service_cb(self, req):
      self.set_mimic(req.mimic, req.speed, req.repeat)
      return SetMimicResponse()

  def action_cb(self, goal):
      if self.set_mimic(goal.mimic, goal.speed, goal.repeat):
          self._as.set_succeeded()
      else:
          self._as.set_aborted()

  def set_mimic(self, mimic, speed, repeat):
      self.ServiceCalled = True
      #os.system(self.quit_command)

      print "Mimic: %s" % mimic
      file_location = '/tmp/mimic/' + mimic + '.mp4'
      if(not os.path.isfile(file_location)):
        print "File not found: " + file_location
        return False
        
      if (repeat == 0 ):
        self.default_mimic = mimic
        self.default_speed = speed
        self.ServiceCalled = False
      else: 
        for i in range(0,repeat):
          command = "export DISPLAY=:0 && vlc --video-wallpaper --video-filter 'rotate{angle=0}' --one-instance --playlist-enqueue --no-video-title-show --rate %f %s"  % (speed,file_location) #--fullscreen --video-filter 'rotate{angle=90}'
          os.system(command)
      os.system(self.quit_command)
      return True

  def defaultMimic(self):
    while not rospy.is_shutdown():
      file_location = '/tmp/mimic/' + self.default_mimic + '.mp4'
      if(not os.path.isfile(file_location)):
        print "File not found: " + file_location
        rospy.sleep(1)
        return
      command = "export DISPLAY=:0 && vlc --video-wallpaper --video-filter 'rotate{angle=0}' --loop --one-instance --playlist-enqueue --no-video-title-show --rate %f  %s"  % (self.default_speed,file_location) # --fullscreen --video-filter 'rotate{angle=90}'
      os.system(command)

      
      
  def main(self):
    rospy.init_node('mimic')
    self.ServiceCalled = False
    self.default_speed = 1.0
    self.default_mimic = "default"
    self.quit_command = "export DISPLAY=:0 && vlc --one-instance --playlist-enqueue vlc://quit"
    
    s=rospy.Service('set_mimic',SetMimic, self.service_cb)
    self._as = actionlib.SimpleActionServer('set_mimic_action', cob_mimic.msg.SetMimicAction, execute_cb=self.action_cb, auto_start = False)
    self._as.start()
    
    # copy all videos to /tmp
    rospy.loginfo("copying all mimic files to /tmp/mimic...")
    file_location = roslib.packages.get_pkg_dir('cob_mimic') + '/common/*.mp4'
    os.system("mkdir -p /tmp/mimic")
    os.system("cp " + file_location + " /tmp/mimic")
    rospy.loginfo("...copied all mimic files to /tmp/mimic")
    
    while ( self.ServiceCalled == False):
      self.defaultMimic()
      rospy.sleep(1)
        
    rospy.spin()


 
if __name__ == "__main__":
  try:
    mimic = Mimic()
    mimic.main()
  except KeyboardInterrupt, e:
    pass
  print "exiting"

