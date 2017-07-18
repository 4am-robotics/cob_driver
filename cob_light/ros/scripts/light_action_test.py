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


import rospy
import actionlib
import random

from cob_light.msg import *

from cob_light.srv import SetLightMode
from cob_light.srv import SetLightModeRequest
from cob_light.srv import SetLightModeResponse

from std_msgs.msg import ColorRGBA

class ActionTestScript(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient("/light_torso/set_light", SetLightModeAction)
        # trying to connect to server
        rospy.logdebug("waiting for /light_torso/set_light action server to start")
        if not self.client.wait_for_server(rospy.Duration(5)):
            # error: server did not respond
            rospy.logerr("/light_torso/set_light action server not ready within timeout, aborting...")
        else:
            rospy.logdebug("/light_torso/set_light action server ready")

        self.executeMode(0)

    def executeMode(self, event):
        rospy.loginfo("setting action mode")

        mode = self.getLightMode(random.randint(0,14))
        goal = SetLightModeGoal()
        goal.mode = mode.mode

        self.client.send_goal(goal)
        self.client.wait_for_result()
        res = self.client.get_result()

        duration = random.uniform(0.0, 0.2)
        self.timer = rospy.Timer(rospy.Duration(duration), self.executeMode, True)

    def getLightMode(self, mode):
        req = SetLightModeRequest()
        req.mode.priority = 4

        if mode == 0:
            req.mode.mode=LightModes.BREATH
            req.mode.frequency = 0.3
            req.mode.pulses = 2
            req.mode.timeout = 0
            req.mode.colors.append(ColorRGBA(1.0,1.0,1.0,1.0))

        elif mode == 1:
            req.mode.mode=LightModes.FLASH
            req.mode.frequency = 2
            req.mode.pulses = 3
            req.mode.timeout = 0
            req.mode.colors.append(ColorRGBA(1.0,1.0,1.0,1.0))

        elif mode == 2:
            req.mode.mode=LightModes.BREATH
            req.mode.frequency = 0.4
            req.mode.pulses = 0
            req.mode.timeout = 0
            req.mode.colors.append(ColorRGBA(170.0/255.0, 250.0/255.0, 70.0/255.0, 1.0))

        elif mode == 3:
            req.mode.mode=LightModes.BREATH
            req.mode.frequency = 0.3
            req.mode.pulses = 0
            req.mode.timeout = 0
            req.mode.colors.append(ColorRGBA(70.0/255.0, 140.0/255.0, 250.0/255.0, 1.0))

        elif mode == 4:
            req.mode.mode=LightModes.BREATH
            req.mode.frequency = 0.3
            req.mode.pulses = 0
            req.mode.timeout = 0
            req.mode.colors.append(ColorRGBA(240.0/255.0, 250.0/255.0, 70.0/255.0, 1.0))

        elif mode == 5:
            req.mode.mode=LightModes.BREATH
            req.mode.frequency = 0.3
            req.mode.pulses = 0
            req.mode.timeout = 0
            req.mode.colors.append(ColorRGBA(70.0/255.0, 240.0/255.0, 250.0/255.0, 1.0))

        elif mode == 6: #static yellow black - barrier tape
            yellow = ColorRGBA(70.0/255.0, 240.0/255.0, 250.0/255.0, 1.0)
            black = ColorRGBA()
            req.mode.mode=LightModes.BREATH
            req.mode.frequency = 0.3
            req.mode.pulses = 0
            req.mode.timeout = 0
            #58 leds
            num_segs = 6
            iterations = int(num_segs/2)
            seg_div = int(58/num_segs)
            for i in range(0, iterations):
                for j in range (0, seg_div):
                    req.mode.colors.append(yellow)
                for j in range (0, seg_div):
                    req.mode.colors.append(black)

        elif mode == 7:
            req.mode.mode=LightModes.BREATH
            req.mode.frequency = 0.3
            req.mode.pulses = 0
            req.mode.timeout = 0
            req.mode.colors.append(ColorRGBA(250.0/255.0, 70.0/255.0, 70.0/255.0, 1.0))

        elif mode == 8:
            req.mode.mode=LightModes.BREATH
            req.mode.frequency = 0.3
            req.mode.pulses = 0
            req.mode.timeout = 0
            req.mode.colors.append(ColorRGBA(230.0/255.0, 250.0/255.0, 250.0/255.0, 1.0))

        elif mode == 9:
            req.mode.mode=LightModes.FADE_COLOR
            req.mode.frequency = 0.5
            req.mode.pulses = 0
            req.mode.timeout = 0
            req.mode.colors.append(ColorRGBA(1,1,1,1))

        elif mode == 10:
            req.mode.mode=LightModes.FLASH
            req.mode.frequency = 4
            req.mode.pulses = 0
            req.mode.timeout = 0
            req.mode.colors.append(ColorRGBA(230.0/255.0, 250.0/255.0, 250.0/255.0, 1.0))

        elif mode == 11:
            req.mode.mode=LightModes.STATIC
            req.mode.frequency = 1
            req.mode.pulses = 0
            req.mode.timeout = 0
            req.mode.colors.append(ColorRGBA(150.0/255.0, 120.0/255.0, 255.0/255.0, 1.0))

        elif mode == 12:
            req.mode.mode=LightModes.STATIC
            req.mode.frequency = 1
            req.mode.pulses = 0
            req.mode.timeout = 0
            req.mode.colors.append(ColorRGBA(255.0/255.0, 95.0/255.0, 1.0/255.0, 1.0))

        elif mode == 13: #breath fast in red (error)
            req.mode.mode=LightModes.BREATH
            req.mode.frequency = 0.8
            req.mode.pulses = 0
            req.mode.timeout = 0
            req.mode.colors.append(ColorRGBA(1.0, 0.0, 0.0, 1.0))

        elif mode == 14: #breath fast in red (error)
            req.mode.mode=LightModes.XMAS
            req.mode.frequency = 0.8
            req.mode.pulses = 0
            req.mode.timeout = 0
            req.mode.colors.append(ColorRGBA(1.0, 0.0, 0.0, 1.0))

        return req


if __name__ == '__main__':
  rospy.init_node('light_action_test')
  ActionTestScript()
  rospy.spin()
