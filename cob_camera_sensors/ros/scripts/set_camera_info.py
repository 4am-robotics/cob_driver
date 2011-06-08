#!/usr/bin/python

import roslib
roslib.load_manifest('cob_camera_sensors')
import rospy
from sensor_msgs.msg import *
from sensor_msgs.srv import *

rospy.init_node('set_camera_info')
rospy.wait_for_service('camera/set_camera_info')
set_camera_info = rospy.ServiceProxy('camera/set_camera_info', SetCameraInfo)
req = SetCameraInfoRequest()
req.camera_info.header.stamp = rospy.Time.now()
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
