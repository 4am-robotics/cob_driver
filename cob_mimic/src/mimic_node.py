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
import os
import subprocess
import vlc

import roslib
import rospy
import actionlib

from cob_mimic.srv import *
from cob_mimic.msg import *

class Mimic:
    def __init__(self):
        self.media =                None
        self.player =               None
        self.playlist =             None
        self.repeat =               None
        self.emotion =              None
        self.vlc_instance =         None
        self.playback_speed =       None
        self.default_mimic =        'default'
        self.default_mimic_file =   '/tmp/mimic/' + self.default_mimic +'.mp4'
        #lets start the default emotion
        self.setup_playback()

    def service_cb(self, req):
        success = self.set_mimic(req.mimic, req.speed, req.repeat)
        return SetMimicResponse(success, "")

    def action_cb(self, goal):
        if self.set_mimic(goal.mimic, goal.speed, goal.repeat):
            self._as.set_succeeded()
        else:
            self._as.set_aborted()

    def set_mimic(self, mimic, speed, repeat):
        rospy.loginfo("Mimic: %s", mimic)
        file_location = '/tmp/mimic/' + mimic + '.mp4'
        print mimic
        if(not os.path.isfile(file_location)):
            rospy.logerr("File not found ")
            return False
        # repeat cannot be 0
        repeat = max (1, repeat)
        self.emotion = mimic
        self.playback_speed = speed
        self.repeat = repeat
        self.set_emotion()
        return True

    def main(self):
        self.rotation = rospy.get_param('~rotation', 0)
        # copy all videos to /tmp
        rospy.loginfo("copying all mimic files to /tmp/mimic...")
        file_location = roslib.packages.get_pkg_dir('cob_mimic') + '/common/*.mp4'
        os.system("mkdir -p /tmp/mimic")
        os.system("cp " + file_location + " /tmp/mimic")
        rospy.loginfo("...copied all mimic files to /tmp/mimic")
        self._ss = rospy.Service('~set_mimic', SetMimic, self.service_cb)
        self._as = actionlib.SimpleActionServer('~set_mimic', cob_mimic.msg.SetMimicAction, execute_cb=self.action_cb, auto_start = False)
        self._as.start()
        self.play_defualt_emotion()
        rospy.spin()

    def setup_playback(self):
        try:
            if not os.path.isfile(self.default_mimic_file):
                rospy.logerr("File not found: %s", self.default_mimic_file)
                return
            # run single instance of the media player
            self.vlc_instance = vlc.Instance('--ignore-config', '--mouse-hide-timeout=0', '-q', '--no-osd', '-L', '--one-instance', '--playlist-enqueue', '--no-video-title-show')
            self.player = self.vlc_instance.media_player_new()
            self.player.set_fullscreen(int(True))
        except Exception as e:
            rospy.logerr("Something went wrong while setting up media playback: %s", str(e))

    def play_defualt_emotion(self):
        self.media = self.vlc_instance.media_new(self.default_mimic_file)
        self.vlc_instance.vlm_set_loop(self.default_mimic_file, int(True))
        self.media.get_mrl()
        self.playback_speed = 1.0 # default playback speed 1.0
        self.player.set_rate(float(self.playback_speed))
        self.player.set_media(self.media)
        self.player.play()

    def set_emotion(self):
        if self.repeat and self.emotion:
            self.player.set_rate(float(self.playback_speed))
            file_location = '/tmp/mimic/' + self.emotion + '.mp4'
            while self.repeat:
                self.media = self.vlc_instance.media_new(file_location)
                self.media.get_mrl()
                self.player.set_media(self.media)
                self.player.play()
                rospy.sleep(0.1)
                while self.player.is_playing() == 1:
                    rospy.sleep(0.1)
                self.repeat -= 1
            self.repeat = None
            self.emotion = None    
        self.play_defualt_emotion()

if __name__ == "__main__":
    rospy.init_node('mimic')
    try:
        mimic = Mimic()
        mimic.main()
    except (rospy.ROSInterruptException, KeyboardInterrupt, SystemExit) as e:
        rospy.loginfo('Exiting: ' + str(e))
        pass

