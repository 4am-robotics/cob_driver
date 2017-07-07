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
# ROS stack name: cob_driver
# \note
# ROS package name: cob_mimic
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

import os
import vlc
import threading

import roslib
import rospy
import actionlib

from cob_mimic.srv import *
from cob_mimic.msg import *

class Mimic:
    def __init__(self):
        self.mutex = threading.Lock()
        self.new_mimic_request = False

        # copy all mimic files (videos) to /tmp. This is important for all pc 
        # sharing their home directory through a NFS to not copy the file 
        # through the network for each call to mimic.
        rospy.loginfo("copying all mimic files to /tmp/mimic...")
        file_location = roslib.packages.get_pkg_dir('cob_mimic') + '/common/*.mp4'
        os.system("mkdir -p /tmp/mimic")
        os.system("cp " + file_location + " /tmp/mimic")
        rospy.loginfo("...copied all mimic files to /tmp/mimic")

        # specify default mimic
        self.default_mimic = 'default'

        # setup vlc player
        self.vlc_instance = vlc.Instance('--ignore-config', '--mouse-hide-timeout=0', '-q', '--no-osd', '-L', '--one-instance', '--playlist-enqueue', '--no-video-title-show')
        self.player = self.vlc_instance.media_player_new()
        self.player.set_fullscreen(int(True))

        # start the default mimic
        self.set_mimic(self.default_mimic, 1, 1, False)

    def service_cb(self, req):
        success = self.set_mimic(req.mimic, req.speed, req.repeat)
        return SetMimicResponse(success, "")

    def action_cb(self, goal):
        if self.set_mimic(goal.mimic, goal.speed, goal.repeat):
            self._as.set_succeeded()
        else:
            self._as.set_aborted()

    def set_mimic(self, mimic, speed, repeat, blocking = True):
        self.new_mimic_request = True
        rospy.loginfo("New mimic request with %s", mimic)
        self.mutex.acquire()
        self.new_mimic_request = False
        rospy.loginfo("Mimic: %s (speed: %s, repeat: %s)", mimic, speed, repeat)
        file_location = '/tmp/mimic/' + mimic + '.mp4'
        
        # check if mimic exists
        if(not os.path.isfile(file_location)):
            rospy.logerr("File not found: %s", file_location)
            self.mutex.release()
            return False

        # repeat cannot be 0
        repeat = max(1, repeat)

        # speed cannot be 0 or negative
        if speed <= 0:
            rospy.logwarn("Mimic speed cannot be 0 or negative. Setting to 1.0")
            speed = 1.0

        self.player.set_rate(float(speed))
        while repeat:
            media = self.vlc_instance.media_new(file_location)
            media.get_mrl()
            self.player.set_media(media)
            self.player.play()
            rospy.sleep(0.1) # we need to sleep here because is_playing() will not immediately return True after calling play()
            while blocking and self.player.is_playing():
                rospy.sleep(0.1)
                rospy.logdebug("still playing %s", mimic)
                if self.new_mimic_request:
                    rospy.logwarn("mimic %s preempted", mimic)
                    self.mutex.release()
                    return False
                    
            repeat -= 1
        self.mutex.release()
        return True

    def main(self):
        self._ss = rospy.Service('~set_mimic', SetMimic, self.service_cb)
        self._as = actionlib.SimpleActionServer('~set_mimic', cob_mimic.msg.SetMimicAction, execute_cb=self.action_cb, auto_start = False)
        self._as.start()
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('mimic')
    try:
        mimic = Mimic()
        mimic.main()
    except (rospy.ROSInterruptException, KeyboardInterrupt, SystemExit) as e:
        rospy.loginfo('Exiting: ' + str(e))
        pass

