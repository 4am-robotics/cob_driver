#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_script_server')

#import kinematics_msgs.msg
#from kinematics_msgs.msg import *
#import kinematics_msgs.srv
#from kinematics_msgs.srv import *                TODO ADD new emplacement for  GetKinematicSolverInfo

import unittest

import rospy
import rostest

from simple_script_server import *
from cob_srvs.srv import *

import random

###################################################################  Class test
class PythonAPITest(unittest.TestCase):

	# init node
	def __init__(self, *args):
		super(PythonAPITest, self).__init__(*args)
		rospy.init_node('test_kdl_solver',anonymous=True)
		self.ns_global_prefix = "test_kdl_solver"
		
	def test_python_api(self):


						


		###############################################################################################	generate random fct
		def gen_rand(mini,maxi):

  			rand_num = random.random()*100+1;
 			result = mini + ((maxi-mini)*rand_num)/101.0;
 			return result

		rospy.sleep(2)
		
		###############################################################################################	/arm_kinematics/get_ik_solver_info
		if not rospy.wait_for_service('/arm_kinematics/get_ik_solver_info',5):
			self.fail("Service /arm_kinematics/get_ik_solver_info not available")
		

		try:
			get_ik_solver_info = rospy.ServiceProxy('/arm_kinematics/get_ik_solver_info',GetKinematicSolverInfo)  # launch SERVICE
		except Exception, e:
			self.fail("Service /arm_kinematics/get_ik_solver_info unavailable")	

		# call function
		req_Solver_Info = GetKinematicSolverInfoRequest()
		result_SolverInfo = get_ik_solver_info(req_Solver_Info)	
		#rospy.loginfo(result_SolverInfo)#==> debug


		# check results
		if len(result_SolverInfo.kinematic_solver_info.link_names) != 8 :
			rospy.loginfo(len(result_SolverInfo.kinematic_solver_info.link_names))
			self.fail("Service /arm_kinematics/get_fk_solver_info response has not 8 links")

		if len(result_SolverInfo.kinematic_solver_info.joint_names) != 7 :
			rospy.loginfo(len(result_SolverInfo.kinematic_solver_info.link_names))
			self.fail("Service /arm_kinematics/get_fk_solver_info response has not 7 joints")

		###############################################################################################	/arm_kinematics/get_fk_solver_info
	
		rospy.wait_for_service('/arm_kinematics/get_fk_solver_info')
		try:
			get_SolverInfo = rospy.ServiceProxy('/arm_kinematics/get_fk_solver_info',GetKinematicSolverInfo)  # launch SERVICE
		except Exception, e:
			self.fail("Service /arm_kinematics/get_fk_solver_info unavailable")

		# call function
		req_Solver_Info = GetKinematicSolverInfoRequest()
		result_SolverInfo = get_SolverInfo(req_Solver_Info)	
		#rospy.loginfo(result_SolverInfo)#==> debug


		# check results
		if len(result_SolverInfo.kinematic_solver_info.link_names) != 8 :
			rospy.loginfo(len(result_SolverInfo.kinematic_solver_info.link_names))
			self.fail("Service /arm_kinematics/get_fk_solver_info response has not 8 links")

		if len(result_SolverInfo.kinematic_solver_info.joint_names) != 7 :
			rospy.loginfo(len(result_SolverInfo.kinematic_solver_info.link_names))
			self.fail("Service /arm_kinematics/get_fk_solver_info response has not 7 joints")
		



		###############################################################################################	/arm_kinematics/get_fk
		
		rospy.wait_for_service('/arm_kinematics/get_fk')
		try:
			get_fk = rospy.ServiceProxy('/arm_kinematics/get_fk',GetPositionFK)  # launch SERVICE
		except Exception, e:
			self.fail("Service /arm_kinematics/get_fk unavailable")	

		# generate fake data 
		num_joints = len(result_SolverInfo.kinematic_solver_info.joint_names)
		i = 0
		min_limits = []
		max_limits = []
  		while  i < num_joints:
   			min_limits.append(-math.pi)
  			max_limits.append(math.pi)
			i += 1
  
		# fill arguments
		fk_request = GetPositionFKRequest()
		fk_request.header.frame_id = "base_link";
		#rospy.loginfo(fk_request.fk_link_names)#==> debug
		fk_request.fk_link_names = result_SolverInfo.kinematic_solver_info.link_names
 	 	#rospy.loginfo(fk_request.fk_link_names)#==> debug
		fk_request.robot_state.joint_state.name = result_SolverInfo.kinematic_solver_info.joint_names
		#rospy.loginfo(fk_request.robot_state.joint_state.position)#==> debug

		i = 0
  		while  i < num_joints:
    			fk_request.robot_state.joint_state.position.append(gen_rand(min_limits[i],max_limits[i]))
  			i += 1
		#rospy.loginfo(fk_request)#==> debug

		# call function
		try:
			fk_response = get_fk(fk_request)
		except Exception, e:
			self.fail("Function calling fail : /arm_kinematics/get_fk")	
		rospy.loginfo(fk_response)#==> debug
		
		# check results
		if fk_response.error_code.val != 1:
			self.fail('Function fail /arm_kinematics/get_fk error code != 1')
		if len(fk_response.pose_stamped) != 8:
			self.fail("Service /arm_kinematics/get_fk_solver_info response has not 8 poses")
		if len(fk_response.fk_link_names) != 8:
			self.fail("Service /arm_kinematics/get_fk_solver_info response has not 8 link names")



		###############################################################################################	/arm_kinematics/get_ik 
		rospy.wait_for_service('/arm_kinematics/get_ik')
		

		try:
			get_ik = rospy.ServiceProxy('/arm_kinematics/get_ik',GetPositionIK)  # launch SERVICE
		except Exception, e:
			self.fail("Service /arm_kinematics/get_ik unavailable")	
		
		req = GetPositionIKRequest()
		
		req.ik_request.pose_stamped = fk_response.pose_stamped[7];
		req.ik_request.ik_seed_state.joint_state.name = result_SolverInfo.kinematic_solver_info.joint_names;
		req.ik_request.ik_link_name = result_SolverInfo.kinematic_solver_info.link_names[7]
		req.ik_request.ik_seed_state.joint_state.position = [0]*7
		
		nb_error = -1
		success = 0


		while success != 1 and nb_error < 10:
			req.ik_request.robot_state.joint_state.position = []
			fk_request.robot_state.joint_state.position = []
			i = 0
  			while  i < num_joints:
    				fk_request.robot_state.joint_state.position.append(gen_rand(min_limits[i],max_limits[i]))
  				i += 1
			try:
				fk_response = get_fk(fk_request)
			except Exception, e:
				self.fail("Function calling fail : /arm_kinematics/get_fk")	
			req.ik_request.pose_stamped = fk_response.pose_stamped[7];
			i = 0
	  		while  i < num_joints:
	    			req.ik_request.robot_state.joint_state.position.append(gen_rand(min_limits[i],max_limits[i]))
	  			i += 1
			#rospy.loginfo(req)#==> debug
			
			
			try:
				ik_response = get_ik(req)
			except Exception, e:
				self.fail("Function calling fail : /arm_kinematics/get_ik")
			nb_error += 1
			success = ik_response.error_code.val
			#rospy.loginfo(ik_response)#==> debug

		# check results
		if ik_response.error_code.val != 1:
			self.fail('Function fail /arm_kinematics/get_fk error code != 1')
		if len(ik_response.solution.joint_state.name) != 7:
			self.fail('Function fail /arm_kinematics/get_fk error no 7 joint names')
		if len(ik_response.solution.joint_state.position) != 7:
			self.fail('Function fail /arm_kinematics/get_fk error no 7 position results')

		###############################################################################################	/arm_kinematics/get_constraint_aware_ik
		rospy.wait_for_service('/arm_kinematics/get_constraint_aware_ik')
		

		try:
			get_constraint_aware_ik = rospy.ServiceProxy('/arm_kinematics/get_constraint_aware_ik',GetConstraintAwarePositionIK)  # launch SERVICE
		except Exception, e:
			self.fail("Service /arm_kinematics/get_constraint_aware_ik unavailable")

			
		req_constraint_aware_ik = GetConstraintAwarePositionIKRequest()
		req_constraint_aware_ik.ik_request = req.ik_request
		response = get_constraint_aware_ik(req_constraint_aware_ik)
		#rospy.loginfo(response)#==> debug

		

		
		# check results
		if ik_response.error_code.val != 1:
			self.fail('Function fail /arm_kinematics/get_constraint_aware_ik error code != 1')
		if len(response.solution.joint_state.name) != 7:
			self.fail('Function fail /arm_kinematics/get_constraint_aware_ik error no 7 joint names')
		if len(response.solution.joint_state.position) != 7:
			self.fail('Function fail /arm_kinematics/get_constraint_aware_ik error no 7 position results')

		############################################################################################### Stop rospy						
    		def myhook(): 
  			print "shutdown time!"
		rospy.on_shutdown(myhook)
			
################################################################### start the rostest
if __name__ == '__main__':
	try:
		rostest.run('rostest', 'kdl_solver', PythonAPITest, sys.argv)
	except KeyboardInterrupt, e:
		pass
	print "exiting"

