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
		#self.service = rospy.Service("move_cart_rel", MoveCart, self.cbIKSolverRel)
		self.client = actionlib.SimpleActionClient('joint_trajectory_action', JointTrajectoryAction)
		self._as = actionlib.SimpleActionServer("move_cart_rel", MoveCartAction, execute_cb=self.cbIKSolverRel)
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
		result = MoveCartResult()
		feedback = MoveCartFeedback()
		try:
			(trans,rot) = self.listener.lookupTransform('base_link', 'arm_7_link', rospy.Time(0))
		except tf.LookupException as lex:
			print "Error Lookup"
			print lex
		except tf.ConnectivityException as cex:
			print "Error Connectivity"
			print cex
		
		relpos = Pose()
		relpos.position.x = trans[0] + msg.goal_pose.pose.position.x
		relpos.position.y = trans[1] + msg.goal_pose.pose.position.y
		relpos.position.z = trans[2] + msg.goal_pose.pose.position.z
		#angles_cmd = tf.transformations.euler_from_quaternion([msg.goal_pose.pose.orientation.x, msg.goal_pose.pose.orientation.y, msg.goal_pose.pose.orientation.z, msg.goal_pose.pose.orientation.w])
		#angles_cur = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
		
		#qrel = tf.transformations.quaternion_from_euler(angles_cmd[0]+angles_cur[0], angles_cmd[1]+angles_cur[1], angles_cmd[2]+angles_cur[2])

		#check for uninitialized msg
		if(msg.goal_pose.pose.orientation.x == 0.0 and msg.goal_pose.pose.orientation.y == 0.0 and msg.goal_pose.pose.orientation.z == 0.0 and msg.goal_pose.pose.orientation.w == 0.0):
			msg.goal_pose.pose.orientation.w = 1.0
		qrel = tf.transformations.quaternion_multiply([msg.goal_pose.pose.orientation.x, msg.goal_pose.pose.orientation.y, msg.goal_pose.pose.orientation.z, msg.goal_pose.pose.orientation.w], [rot[0], rot[1], rot[2], rot[3]])
		print qrel
		relpos.orientation.x = qrel[0]
		relpos.orientation.y = qrel[1]
		relpos.orientation.z = qrel[2]
		relpos.orientation.w = qrel[3]
		(new_config, error) = self.callIKSolver(relpos)
		if(error != -1):
			self.moveArm(new_config)
			result.return_value = 0
			self._as.set_succeeded(result)
		else:
			result.return_value = -1
			self.as_.setAborted(result);
	
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
		self.client.wait_for_result()

	def callIKSolver(self, goal_pose):
		while(not self.received_state):
			time.sleep(0.1)
		req = GetPositionIKRequest()
		req.ik_request.ik_seed_state.joint_state.position = self.configuration
		req.ik_request.pose_stamped.pose = goal_pose
		resp = self.iks(req)
		return (resp.solution.joint_state.position, resp.error_code.val)
	
		
		
		



if __name__ == "__main__":
	rospy.init_node('ik_solver_test')
	time.sleep(0.5)
	iks = ik_solver()
	rospy.spin()
