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
from cob_phidgets.msg import AnalogSensor
from cob_phidgets.msg import DigitalSensor
from cob_msgs.msg import PowerState
from cob_msgs.msg import EmergencyStopState

class EmState():
    ST_EM_FREE = 0
    ST_EM_ACTIVE = 1
    ST_EM_CONFIRMED = 2

class EMStatePhidget():
    def __init__(self):
        self.last_update = None
        self.em_caused_by_button = False

        self.last_rear_em_state = False
        self.last_front_em_state = False

        self.em_stop_status = EmState.ST_EM_ACTIVE
        self.em_msg = EmergencyStopState()

        self.sub_digital_sensors = rospy.Subscriber("digital_sensors", DigitalSensor, self.phidget_cb)
        self.pub_em_state = rospy.Publisher('em_stop_state', EmergencyStopState, queue_size=1)

    def phidget_cb(self, msg):
        front_em_active = False
        rear_em_active = False
        got_message = False

        for i in range(0, len(msg.uri)):
            if msg.uri[i] == "em_stop_laser_rear":
                rear_em_active = not msg.state[i]
                got_message = True
            if msg.uri[i] == "em_stop_laser_front":
                front_em_active = not msg.state[i]
                got_message = True

        if got_message:
            self.last_update = rospy.Time.now()
            
            if (front_em_active and rear_em_active) and (not self.last_front_em_state and  not self.last_rear_em_state):
                self.em_msg.emergency_button_stop = True
                self.em_caused_by_button = True
            elif (not front_em_active and not rear_em_active) and (self.last_front_em_state and self.last_rear_em_state):
                self.em_msg.emergency_button_stop = False
                self.em_caused_by_button = False
            elif (front_em_active is not rear_em_active) and self.em_caused_by_button :
                self.em_msg.emergency_button_stop = False
                self.em_caused_by_button = False
                self.em_msg.scanner_stop = (front_em_active or rear_em_active)
            else:
                self.em_msg.scanner_stop = (front_em_active or rear_em_active);

            em_signal = self.em_msg.scanner_stop or self.em_msg.emergency_button_stop

            if self.em_stop_status == EmState.ST_EM_FREE:
                if em_signal:
                    rospy.logdebug("Emergency stop was issued")
                    self.em_stop_status = EmergencyStopState.EMSTOP

            elif self.em_stop_status == EmState.ST_EM_ACTIVE:
                if not em_signal:
                    rospy.logdebug("Emergency stop was confirmed")
                    self.em_stop_status = EmergencyStopState.EMCONFIRMED
            elif self.em_stop_status == EmState.ST_EM_CONFIRMED:
                if em_signal:
                    rospy.logdebug("Emergency stop issued")
                    self.em_stop_status = EmergencyStopState.EMSTOP
                else:
                    rospy.logdebug("Emergency stop released")
                    self.em_stop_status = EmergencyStopState.EMFREE

            self.em_msg.emergency_state = self.em_stop_status

            self.last_rear_em_state = rear_em_active
            self.last_front_em_state = front_em_active

    def publish(self):
        if self.last_update is not None:
            self.pub_em_state.publish(self.em_msg)


if __name__ == "__main__":
    rospy.init_node("em_state_phidget")
    emsp = EMStatePhidget()
    rospy.loginfo("em_state_phidget running")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        emsp.publish()
        rate.sleep()
