#!/usr/bin/env python
import serial
import roslib; 
roslib.load_manifest('cob_hardware_info')
import rospy
from diagnotic_msgs.msg import * 

ns_global_prefix = "/battery_state"
rospy.init_node('battery_state')
#read in paramterers
if not rospy.has_param(ns_global_prefix + "/devicestring"):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",ns_global_prefix + "/devicestring")
			sys.exit()
devicestring_param = rospy.get_param(self.ns_global_prefix + "/devicestring")
if not rospy.has_param(ns_global_prefix + "/minimum_battery_voltage"):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",ns_global_prefix + "/minimum_battery_voltage")
			sys.exit()
minvalue_param = rospy.get_param(self.ns_global_prefix + "/minimum_battery_voltage")
if not rospy.has_param(ns_global_prefix + "/maximum_battery_voltage"):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",ns_global_prefix + "/maximum_battery_voltage")
			sys.exit()
maxvalue_param = rospy.get_param(self.ns_global_prefix + "/maximum_battery_voltage")

diagnostic_publisher = rospy.Publisher("/diagnostics", DiagnosticStatus)
batcon = serial.Serial("/dev/ttyUSB2", 230400)
msg = DiagnosticStatus()
msg.level = 0
msg.name = "battery_state"
msg.message = "measuring battery voltage"
valuevoltage = KeyValue()
valuevoltage.key = "Voltage"
valuevoltage.value = "0"
msg.values.append(valuevoltage)
valuepercentage = KeyValue()
valuepercentage.key = "Percentage"
valuepercentage.value = "0"
msg.values.append(valuevoltage)


while not rospy.is_shutdown():
	batcon.write("B\r")
	batvalue = batcon.readline()
	diagnostic_publisher.publish(msg)
	

