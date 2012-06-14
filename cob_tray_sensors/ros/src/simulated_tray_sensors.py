#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_tray_sensors')
import rospy
import math
from gazebo_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

class GazeboVirtualRangeSensor():
	
	def __init__(self, sensor_nr, sim_topic_name, topic_name):		
		rospy.Subscriber(sim_topic_name, LaserScan, self.laser_callback)
		rospy.logdebug("subscribed to bumper states topic: %s", topic_name)
		self.pub = rospy.Publisher(topic_name, Range)
		rospy.loginfo("topic '" + topic_name + "' advertised")
		self.range = Range()

	def laser_callback(self, msg):
		# set header
		self.range.header = msg.header
		
		# set range from average out of laser message
		if len(msg.ranges) > 0:
			self.range.range = sum(msg.ranges)/len(msg.ranges)
		else:
			self.range.range = 0
		
		# set min and max range
		self.range.min_range = msg.range_min
		self.range.max_range = msg.range_max
		
		# set field of view
		self.range.field_of_view = msg.angle_max - msg.angle_min
		
		# set radiation type to IR (=1)
		self.range.radiation_type = 1

	'''
	Publish the current state of the simulated sensors.
	'''
	def publish(self):
		self.pub.publish(self.range)

if __name__ == "__main__":
	rospy.init_node('tactile_sensors')
	rospy.sleep(0.5)
	
	sensor1 = GazeboVirtualRangeSensor(1, "range_1_sim", "range_1")
	sensor2 = GazeboVirtualRangeSensor(2, "range_2_sim", "range_2")
	sensor3 = GazeboVirtualRangeSensor(3, "range_3_sim", "range_3")
	sensor4 = GazeboVirtualRangeSensor(4, "range_4_sim", "range_4")
	
	r = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		sensor1.publish()
		sensor2.publish()
		sensor3.publish()
		sensor4.publish()
		r.sleep()
