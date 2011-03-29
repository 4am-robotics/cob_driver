#ifndef COB_ARM_IK_SOLVER_H
#define COB_ARM_IK_SOLVER_H

#include <urdf/model.h>
#include <Eigen/Array>
#include <kdl/chainiksolver.hpp>
#include <pr2_arm_kinematics/pr2_arm_kinematics_utils.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/PositionIKRequest.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_kdl.h>



/** @class
 * @brief ROS/KDL based interface for the inverse kinematics of the PR2 arm
 * @author Sachin Chitta <sachinc@willowgarage.com>
 *
 * This class provides a KDL based interface to the inverse kinematics of the PR2 arm. It inherits from the KDL::ChainIkSolverPos class
 * but also exposes additional functionality to return multiple solutions from an inverse kinematics computation.
 */
class CobArmIKSolver
{
	public: // methods

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		/**
		 * Ctor.
		 */
		CobArmIKSolver(const urdf::Model &robot_model, 
						const std::string &root_frame_name,
						const std::string &tip_frame_name,
						const double &search_discretization_angle, 
						const int &free_angle);

		/**
		 * Dtor.
		 */
		~CobArmIKSolver();

		/**
		 * @brief The KDL solver interface that is required to be implemented. NOTE: This method only returns a solution
		 * if it exists for the free parameter value passed in. To search for a solution in the entire workspace use the CartToJntSearch 
		 * method detailed below.
		 *
		 * @param q_init The initial guess for the inverse kinematics solution. The solver uses the joint value q_init(pr2_ik_->free_angle_) as 
		 * as an input to the inverse kinematics. pr2_ik_->free_angle_ can either be 0 or 2 corresponding to the shoulder pan or shoulder roll angle 
		 * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
		 * @param q_out A single inverse kinematic solution (if it exists).  
		 * @return < 0 if no solution is found
		 */
		int CartToJnt(const KDL::JntArray& q_init, 
						const KDL::Frame& p_in, 
						KDL::JntArray& q_out);

		/**
		 * @brief An extension of the KDL solver interface to return all solutions found. NOTE: This method only returns a solution 
		 * if it exists for the free parameter value passed in. To search for a solution in the entire workspace use the CartToJntSearch 
		 * method detailed below.
		 *
		 * @param q_init The initial guess for the inverse kinematics solution. The solver uses the joint value q_init(pr2_ik_->free_angle_) as 
		 * as an input to the inverse kinematics. pr2_ik_->free_angle_ can either be 0 or 2 corresponding to the shoulder pan or shoulder roll angle 
		 * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
		 * @param q_out A std::vector of KDL::JntArray containing all found solutions.  
		 * @return < 0 if no solution is found
		 */
		int CartToJnt(const KDL::JntArray& q_init, 
						const KDL::Frame& p_in, 
						std::vector<KDL::JntArray> &q_out);

		/**
		 * @brief This method searches for and returns the first set of solutions it finds. 
		 *
		 * @return < 0 if no solution is found
		 * @param q_in The initial guess for the inverse kinematics solution. The solver uses the joint value q_init(pr2_ik_->free_angle_) as 
		 * as an input to the inverse kinematics. pr2_ik_->free_angle_ can either be 0 or 2 corresponding to the shoulder pan or shoulder roll angle 
		 * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
		 * @param q_out A std::vector of KDL::JntArray containing all found solutions.  
		 * @param timeout The amount of time (in seconds) to spend looking for a solution.
		 */
		int CartToJntSearch(const KDL::JntArray& q_in, 
							const KDL::Frame& p_in, 
							std::vector<KDL::JntArray> &q_out, 
							const double &timeout);

		/**
		 * @brief This method searches for and returns the closest solution to the initial guess in the first set of solutions it finds. 
		 *
		 * @param q_in The initial guess for the inverse kinematics solution. The solver uses the joint value q_init(pr2_ik_->free_angle_) as 
		 * as an input to the inverse kinematics. pr2_ik_->free_angle_ can either be 0 or 2 corresponding to the shoulder pan or shoulder roll angle 
		 * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
		 * @param q_out A std::vector of KDL::JntArray containing all found solutions.  
		 * @param timeout The amount of time (in seconds) to spend looking for a solution.
		 * @return < 0 if no solution is found
		 */
		int CartToJntSearch(const KDL::JntArray& q_in, 
							const KDL::Frame& p_in, 
							KDL::JntArray &q_out, 
							const double &timeout);

		/**
		 * @brief A method to get chain information about the serial chain that the IK operates on 
		 *
		 * @param response This class gets populated with information about the joints that IK operates on, including joint names and limits.
		 */
		void getSolverInfo(kinematics_msgs::KinematicSolverInfo &response);

		/**
		 * @brief This method searches for and returns the first solution it finds that also satisifies both user defined callbacks.
		 *
		 * @param q_init The initial guess for the inverse kinematics solution. The solver uses the joint value q_init(pr2_ik_->free_angle_) as 
		 * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
		 * @param q_out A std::vector of KDL::JntArray containing all found solutions.  
		 * @param desired_pose_callback A callback function to which the desired pose is passed in
		 * @param solution_callback A callback function to which IK solutions are passed in
		 * @return < 0 if no solution is found
		 */
		int CartToJntSearch(const KDL::JntArray& q_in, 
							const KDL::Frame& p_in, 
							KDL::JntArray &q_out, 
							const double &timeout, 
							motion_planning_msgs::ArmNavigationErrorCodes &error_code,
							const boost::function<void(const KDL::JntArray&,const KDL::Frame&,motion_planning_msgs::ArmNavigationErrorCodes &)> &desired_pose_callback,
							const boost::function<void(const KDL::JntArray&,const KDL::Frame&,motion_planning_msgs::ArmNavigationErrorCodes &)> &solution_callback);


		/**
		 *
		 */
		std::string getFrameId();


	private: // functions
		bool getCount(int &count, const int &max_count, const int &min_count);


	public: // variables and constants
		/**
		 * IK didn't find a solution.
		 */
		static const int NO_IK_SOLUTION;
		
		/**
		 * IK timed out while searching a solution.
		 */
		static const int TIMED_OUT;


	private: // variables and constants
		/**
		 *
		 */
		double search_discretization_angle_;

		/**
		 *
		 */
		int free_angle_;

		/**
		 *
		 */
		std::string root_frame_name_;
		
		/**
		 * The access to ROS.
		 */
		ros::NodeHandle node_handle_;
		
		/**
		 * Information about the IK solver.
		 */
		kinematics_msgs::KinematicSolverInfo solver_info_;
		
		/**
		 * A description of the arm that can be used by KDL to solve the FK and
		 * IK.
		 */
		KDL::Chain chain_;
		
		/**
		 * The number of joints in the KDL chain i.e. the number of joints of
		 * the arm.
		 */
		int number_of_joints_;
		
		/**
		 * The minimum values that the joints of the arm can take before a self
		 * collision of the arm.
		 */
		boost::shared_ptr<KDL::JntArray> q_min_;
		
		/**
		 * The maximum values that the joints of the arm can take before a self
		 * collision of the arm.
		 */
		boost::shared_ptr<KDL::JntArray> q_max_;
};

#endif // COB3_ARM_IK_SOLVER_H

