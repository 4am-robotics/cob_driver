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
from sensor_msgs.msg import *
from sensor_msgs.srv import *

rospy.init_node('set_camera_info')
rospy.wait_for_service('camera/set_camera_info')
set_camera_info = rospy.ServiceProxy('camera/set_camera_info', SetCameraInfo)
req = SetCameraInfoRequest()
req.camera_info.header.stamp = rospy.Time.now()
req.camera_info.header.frame_id = rospy.get_param('~frame_id')
req.camera_info.height = rospy.get_param('~image_height')
req.camera_info.width = rospy.get_param('~image_width')
req.camera_info.distortion_model = "plumb_bob"
req.camera_info.D = rospy.get_param('~distortion_coefficients/data')
req.camera_info.K = rospy.get_param('~camera_matrix/data')
req.camera_info.R = rospy.get_param('~rectification_matrix/data')
req.camera_info.P = rospy.get_param('~projection_matrix/data')
print req.camera_info
try:
  resp1 = set_camera_info(req)
except rospy.ServiceException, e:
  print "Service did not process request: %s"%str(e)
