#!/usr/bin/python

import roslib 
roslib.load_manifest('cob_lookat_controller')
import rospy
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from geometry_msgs.msg import Point, PoseStamped
from std_srvs.srv import Empty
from cob_srvs.srv import SetString
#import cob_lookat_action.msg
import copy
import math
import tf
import actionlib
from copy import deepcopy


class InteractiveLookatTarget():
	def __init__(self):
		#self.client = actionlib.SimpleActionClient('lookat_action', cob_lookat_action.msg.LookAtAction)
		#print "Waiting for Server..."
		##self.client.wait_for_server()
		#print "...done!"
		
		print "Waiting for StartTrackingServer..."
		rospy.wait_for_service('/lookat_controller/start_tracking')
		print "...done!"
		self.start_tracking_client = rospy.ServiceProxy('/lookat_controller/start_tracking', SetString)
		
		print "Waiting for StopTrackingServer..."
		rospy.wait_for_service('/lookat_controller/stop_tracking')
		print "...done!"
		self.stop_tracking_client = rospy.ServiceProxy('/lookat_controller/stop_tracking', Empty)
		
		self.target_pose = PoseStamped()
		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.header.frame_id = "base_link"
		self.target_pose.pose.orientation.w = 1.0
		#self.viz_pub = rospy.Publisher('lookat_target', PoseStamped)
		self.br = tf.TransformBroadcaster()
		self.listener = tf.TransformListener()
		
		transform_available = False
		while not transform_available:
			try:
				(trans,rot) = self.listener.lookupTransform('/base_link', '/lookat_focus_frame', rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				rospy.logwarn("Waiting for transform...")
				rospy.sleep(0.5)
				continue
			transform_available = True
			
		self.target_pose.pose.position.x = trans[0]
		self.target_pose.pose.position.y = trans[1]
		self.target_pose.pose.position.z = trans[2]
		
		self.ia_server = InteractiveMarkerServer("marker_server")
		self.int_marker = InteractiveMarker()
		self.int_marker.header.frame_id = "base_link"
		self.int_marker.pose = self.target_pose.pose
		self.int_marker.name = "lookat_target"
		self.int_marker.description = "virtual lookat target"
		self.int_marker.scale = 0.5

		# create a grey box marker
		box_marker = Marker()
		box_marker.type = Marker.CUBE
		box_marker.scale.x = 0.1
		box_marker.scale.y = 0.1
		box_marker.scale.z = 0.1
		box_marker.color.r = 0.0
		box_marker.color.g = 0.5
		box_marker.color.b = 0.5
		box_marker.color.a = 1.0
		box_control = InteractiveMarkerControl()
		box_control.always_visible = True
		box_control.markers.append( box_marker )
		self.int_marker.controls.append(box_control)

		control = InteractiveMarkerControl()
		control.always_visible = True
		control.orientation.w = 1
		control.orientation.x = 1
		control.orientation.y = 0
		control.orientation.z = 0
		control.name = "move_3D"
		control.interaction_mode = InteractiveMarkerControl.MOVE_3D
		self.int_marker.controls.append(deepcopy(control))
		control.name = "move_x"
		control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
		self.int_marker.controls.append(deepcopy(control))
		control.name = "rotate_x"
		control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
		self.int_marker.controls.append(deepcopy(control))
		control.name = "move_y"
		control.orientation.x = 0
		control.orientation.y = 1
		control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
		self.int_marker.controls.append(deepcopy(control))
		control.name = "rotate_y"
		control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
		self.int_marker.controls.append(deepcopy(control))
		control.name = "move_z"
		control.orientation.y = 0
		control.orientation.z = 1
		control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
		self.int_marker.controls.append(deepcopy(control))
		control.name = "rotate_z"
		control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
		self.int_marker.controls.append(deepcopy(control))
		
		self.ia_server.insert(self.int_marker, self.marker_fb)

		#create menu
		self.menu_handler = MenuHandler()
		#self.menu_handler.insert( "Lookat", callback=self.lookat )
		self.menu_handler.insert( "StartTracking", callback=self.start_tracking )
		self.menu_handler.insert( "StopTracking", callback=self.stop_tracking )
		self.int_marker_menu = InteractiveMarker()
		self.int_marker_menu.header.frame_id = "base_link"
		self.int_marker_menu.name = "lookat_menu"
		self.int_marker_menu.description = "Menu"
		self.int_marker_menu.scale = 1.0
		self.int_marker_menu.pose.position.z = 1.2
		control = InteractiveMarkerControl()
		control.interaction_mode = InteractiveMarkerControl.MENU
		control.description="Lookat"
		control.name = "menu_only_control"
		self.int_marker_menu.controls.append(copy.deepcopy(control))
		self.ia_server.insert(self.int_marker_menu, self.menu_fb)
		self.menu_handler.apply( self.ia_server, self.int_marker_menu.name )
		self.ia_server.applyChanges()

	#def lookat(self, fb):
		#print "lookat pressed, handle at " + str(self.handle_pose.x) + ", " + str(self.handle_pose.y)
		## Creates a goal to send to the action server.
		#goal = cob_lookat_action.msg.LookAtGoal()
		##goal.target = fk_res.pose_stamped[0]
		#focus = PoseStamped()
		#focus.header.stamp = rospy.Time.now()
		#focus.header.frame_id = "base_link"
		#focus.pose.position = self.handle_pose
		#focus.pose.orientation.w = 1.0
		#goal.target = focus
		#self.client.send_goal(goal)
		## Waits for the server to finish performing the action.
		#self.client.wait_for_result()
		#result = self.client.get_result()
		#print result
	
	def start_tracking(self, fb):
		#print "start_tracking pressed, handle at " + str(self.handle_pose.x) + ", " + str(self.handle_pose.y)
		try:
			res = self.start_tracking_client(data='lookat_target')
			print res
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
	def stop_tracking(self, fb):
		#print "stop_tracking pressed, handle at " + str(self.handle_pose.x) + ", " + str(self.handle_pose.y)
		try:
			res = self.stop_tracking_client()
			print res
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
	
	
	def menu_fb(self, fb):
		pass

	def marker_fb(self, fb):
		#p = feedback.pose.position
		#print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)
		self.target_pose.header = fb.header
		self.target_pose.pose = fb.pose
		self.ia_server.applyChanges()

 	def run(self):
 		#self.max_angle = 20
 		#if(self.focus != 4):
 			#focus_transformed = self.listener.transformPose("head_ids_middle_frame",self.focus)
 			#max_dist = math.sqrt(focus_transformed.pose.position.y*focus_transformed.pose.position.y + focus_transformed.pose.position.z * focus_transformed.pose.position.z)
 			#cur_max_angle = math.fabs(math.degrees(math.tan(max_dist/focus_transformed.pose.position.x)))
 			#print cur_max_angle
 			#if(cur_max_angle > self.max_angle and cur_max_angle < 90):
 				#goal = cob_lookat_action.msg.LookAtGoal()
 				#goal.target = self.focus
 				#self.client.send_goal(goal)
				## Waits for the server to finish performing the action.
				#self.client.wait_for_result()
				#result = self.client.get_result()
				#print result
		#self.br.sendTransform((self.handle_pose.x, self.handle_pose.y, self.handle_pose.z), tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(), "prace_handle", "base_link")
		self.br.sendTransform((self.target_pose.pose.position.x, self.target_pose.pose.position.y, self.target_pose.pose.position.z), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "lookat_target", self.target_pose.header.frame_id)
		

if __name__ == "__main__":
	rospy.init_node("interactive_lookat_target")
	ilt = InteractiveLookatTarget()
	#ilt.run()
 	r = rospy.Rate(68)
 	while not rospy.is_shutdown():
   		ilt.run()
		r.sleep()
