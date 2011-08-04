#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_manipulator')
import tf
from kinematics_msgs.srv import *
from sensor_msgs.msg import JointState
import threading
import rospy
import time
import actionlib
from pr2_controllers_msgs.msg import *
from cob_srvs.srv import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import *

class move_cart:

	def __init__(self):
		self.configuration = [0,0,0,0,0,0,0]
		self.lock = threading.Lock()
		self.received_state = False
		self.listener = tf.TransformListener()
		time.sleep(0.5)
		self.client = actionlib.SimpleActionClient('joint_trajectory_action', JointTrajectoryAction)
		self.as_ = actionlib.SimpleActionServer("move_cart", MoveCartAction, execute_cb=self.cbMoveCart)
		if not self.client.wait_for_server(rospy.Duration(15)):
			rospy.logerr("arm action server not ready within timeout, aborting...")
			return
		else:
			rospy.logdebug("arm action server ready")

		rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
		self.thread = threading.Thread(target=self.joint_states_listener)
		self.thread.start()
		rospy.wait_for_service('get_ik')
		self.iks = rospy.ServiceProxy('get_ik', GetPositionIK)
		rospy.loginfo("move cart interface is ready")

	#thread function: listen for joint_states messages
	def joint_states_listener(self):
		rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
		rospy.spin()


	#callback function: when a joint_states message arrives, save the values
	def joint_states_callback(self, msg):
		self.lock.acquire()
		for k in range(7):
			joint_name = "arm_" + str(k+1) + "_joint"
			for i in range(len(msg.name)):
				if(msg.name[i] == joint_name):
					self.configuration[k] = msg.position[i]
		self.name = msg.name
		self.position = msg.position
		self.velocity = msg.velocity
		self.effort = msg.effort
		self.received_state = True
		self.lock.release()

	def cbMoveCart(self,msg):
		result = MoveCartResult()
		feedback = MoveCartFeedback()
		#msg.goal_pose.header.stamp = self.listener.getLatestCommonTime("/arm_0_link",msg.goal_pose.header.frame_id)
		self.listener.waitForTransform("/arm_0_link",msg.goal_pose.header.frame_id,rospy.Time(),rospy.Duration(5))
		print "Calling IK Server"
		(new_config, error_code) = self.callIKSolver(msg.goal_pose)
		if(error_code.val == error_code.SUCCESS):
			self.moveArm(new_config)
			result.return_value = 0
			self.as_.set_succeeded(result)
		else:
			result.return_value = 1
			rospy.logerr("no ik solution found")
			self.as_.set_aborted(result);
	
	def moveArm(self, positions):
		goal = JointTrajectoryGoal()
		goalp = JointTrajectory()
		goalp.joint_names = ["arm_1_joint","arm_2_joint","arm_3_joint","arm_4_joint","arm_5_joint","arm_6_joint","arm_7_joint"]
		point=JointTrajectoryPoint()
		point.positions=positions
		point.time_from_start=rospy.Duration(3)
		goalp.points.append(point)
		goal.trajectory = goalp
		goal.trajectory.header.stamp = rospy.Time.now()
		self.client.send_goal(goal)
		self.client.wait_for_result()

	def callIKSolver(self, goal_pose_stamped):
		while(not self.received_state):
			time.sleep(0.1)
		req = GetPositionIKRequest()
		req.ik_request.ik_seed_state.joint_state.name = ["arm_%d_joint" % (i+1) for i in range(7)]
		req.ik_request.ik_seed_state.joint_state.position = self.configuration
		req.ik_request.pose_stamped = goal_pose_stamped
		req.timeout = rospy.Duration(5)
		resp = self.iks(req)
		print resp.error_code
		return (resp.solution.joint_state.position, resp.error_code)

if __name__ == "__main__":
	rospy.init_node('move_cart')
	time.sleep(0.5)
	mc = move_cart()
	rospy.spin()
