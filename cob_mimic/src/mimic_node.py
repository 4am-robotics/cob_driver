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

import os
import vlc
import threading

import roslib
import rospy
import actionlib

from cob_mimic.srv import *
from cob_mimic.msg import *

class Mimic:
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
      if(not os.path.isfile(file_location)):
        rospy.logerror("File not found: %s", file_location)
        return False

      # repeat cannot be 0
      repeat = max (1, repeat)

      for i in range(0, repeat):
        rospy.loginfo("Repeat: %s, Mimic: %s", repeat, mimic)
        command = "export DISPLAY=:0 && vlc --video-wallpaper --video-filter 'rotate{angle=%d}' --vout glx --one-instance --playlist-enqueue --no-video-title-show --rate %f  %s  vlc://quit"  % (self.rotation, speed, file_location)
        os.system(command)

      return True

  def defaultMimic(self):
    file_location = '/tmp/mimic/' + self.default_mimic + '.mp4'
    if not os.path.isfile(file_location):
      rospy.logerror("File not found: %s", file_location)
      return

    while not rospy.is_shutdown():
      command = "export DISPLAY=:0 && vlc --video-wallpaper --video-filter 'rotate{angle=%d}' --vout glx --loop --one-instance --playlist-enqueue --no-video-title-show --rate %f  %s  vlc://quit"  % (self.rotation, self.default_speed, file_location)
      os.system(command)

  def main(self):
    self.default_speed = 1.0
    self.default_mimic = "default"
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

    rospy.spin()



if __name__ == "__main__":
  rospy.init_node('mimic')
  try:
    mimic = Mimic()
    mimic.main()
  except (rospy.ROSInterruptException, KeyboardInterrupt, SystemExit) as e:
    rospy.loginfo('Exiting: ' + str(e))
    pass

