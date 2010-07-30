#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_arm')
import tf
from kinematics_msgs.srv import *
from sensor_msgs.msg import JointState
import threading
import rospy
import time
import actionlib
from pr2_controllers_msgs.msg import *
from cob_srvs.srv import *
from cob_actions.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import *

class ik_solver:


	def __init__(self):
		if rospy.has_param('JointNames'):
			self.JointNames = rospy.get_param('JointNames')
		else:
			print " !!!!!!!!!!! JointNames not available !!!!!!!!!!!!!!!!"
			return
		self.lock = threading.Lock()
		self.received_state = False
		self.listener = tf.TransformListener()
		time.sleep(0.5)
		self.service = rospy.Service("move_cart_abs", MoveCart, self.cbIKSolverAbs)
		self.service = rospy.Service("move_cart_rel", MoveCart, self.cbIKSolverRel)
		self.client = actionlib.SimpleActionClient('joint_trajectory_action', JointTrajectoryAction)
		if not self.client.wait_for_server(rospy.Duration(15)):
			rospy.logerr("arm action server not ready within timeout, aborting...")
			return
		else:
			rospy.logdebug("arm action server ready")

		rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
		#self.thread = threading.Thread(target=self.joint_states_listener)
		#self.thread.start()
		rospy.wait_for_service('get_ik')
		self.iks = rospy.ServiceProxy('get_ik', GetPositionIK)		


	#thread function: listen for joint_states messages
	def joint_states_listener(self):
		print "joint_states_listener"
		rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
		rospy.spin()


	#callback function: when a joint_states message arrives, save the values
	def joint_states_callback(self, msg):
		self.lock.acquire()
		self.configuration = [0,0,0,0,0,0,0]
		for k in range(7):
			for i in range(len(msg.name)):
				joint_name = "arm_" + str(k+1) + "_joint"
				if(msg.name[i] == joint_name):
					self.configuration[k] = msg.position[i]
		self.name = msg.name
		self.position = msg.position
		self.velocity = msg.velocity
		self.effort = msg.effort
		self.received_state = True
		self.lock.release()

	def cbIKSolverAbs(self, msg):
		new_config = self.callIKSolver(msg.goal_pose.pose)
		self.moveArm(new_config)

	
	def cbIKSolverRel(self, msg):
		print "cbIKSolverRel"
		try:
			(trans,rot) = self.listener.lookupTransform('base_link', 'arm_7_link', rospy.Time(0))
			print trans
			print rot
		except tf.LookupException as lex:
	        	print "Error"
			print lex
		except tf.ConnectivityException as cex:
			print "Error Connectivity"
			print cex
		relpos = Pose()
		relpos.position.x = trans[0] + msg.goal_pose.pose.position.x
		relpos.position.y = trans[1] + msg.goal_pose.pose.position.y
		relpos.position.z = trans[2] + msg.goal_pose.pose.position.z
		relpos.orientation.x = rot[0] + msg.goal_pose.pose.orientation.x
		relpos.orientation.y = rot[1] + msg.goal_pose.pose.orientation.y
		relpos.orientation.z = rot[2] + msg.goal_pose.pose.orientation.z
		relpos.orientation.w = rot[3] + msg.goal_pose.pose.orientation.w
		new_config = self.callIKSolver(relpos)
		self.moveArm(new_config)
	
	def moveArm(self, pose):
		goal = JointTrajectoryGoal()
		goalp = JointTrajectory()
		goalp.joint_names = ["arm_1_joint","arm_2_joint","arm_3_joint","arm_4_joint","arm_5_joint","arm_6_joint","arm_7_joint"]
		point=JointTrajectoryPoint()
		point.positions=pose
		point.time_from_start=rospy.Duration(3)
		goalp.points.append(point)
		goal.trajectory = goalp
		goal.trajectory.header.stamp = rospy.Time.now()
		self.client.send_goal(goal)

	def callIKSolver(self, goal_pose):
		while(not self.received_state):
			time.sleep(0.1)
		print "waited for service"
		req = GetPositionIKRequest()
		req.ik_request.ik_seed_state.joint_state.position = self.configuration
		req.ik_request.pose_stamped.pose = goal_pose
		resp = self.iks(req)
		return resp.solution.joint_state.position
	
		
		
		



if __name__ == "__main__":
	rospy.init_node('ik_solver_test')
	time.sleep(0.5)
	iks = ik_solver()
	rospy.spin()
