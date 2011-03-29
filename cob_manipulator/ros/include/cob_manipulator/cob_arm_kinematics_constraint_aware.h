#ifndef COB_ARM_IK_CONSTRAINT_AWARE_H
#define COB_ARM_IK_CONSTRAINT_AWARE_H

#include <angles/angles.h>
#include <pr2_arm_kinematics/pr2_arm_kinematics_utils.h>

//Pose command for the ik node
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionFK.h>

#include <boost/shared_ptr.hpp>

#include <planning_environment/monitors/planning_monitor.h>
#include <planning_models/kinematic_model.h>

#include <motion_planning_msgs/DisplayTrajectory.h>
#include <motion_planning_msgs/LinkPadding.h>
#include <motion_planning_msgs/ArmNavigationErrorCodes.h>

#include <kdl/chainfksolverpos_recursive.hpp>

#include "cob_manipulator/cob_arm_ik_solver.h"



/** @class
 * @brief ROS/KDL based interface for the inverse kinematics of the COB arm
 * @author Sachin Chitta <sachinc@willowgarage.com>
 * @author Sven Schneider <sven.schneider@smail.inf.h-brs.de>
 *
 * This class provides a ROS/KDL interface to the inverse kinematics of the COB arm. 
 * It will compute a collision free solution to the inverse kinematics of the COB arm. 
 * The collision environment needs to be active for this method to work. This requires the presence of a node 
 * that is publishing collision maps. 
 * To use this node, you must have a roscore running with a robot description available from the ROS param server. 
 */
class CobArmIKConstraintAware
{
	public: // methods
		/**
		 * Ctor.
		 */
		CobArmIKConstraintAware();
		
		/**
		 * Dtor.
		 */
		virtual ~CobArmIKConstraintAware();

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
							const double &timeout,
							motion_planning_msgs::ArmNavigationErrorCodes& error_code);

		/**
		 * @brief This method searches for and returns the closest solution to the initial guess in the first set of solutions it finds.
		 *
		 * This is the service method that is published by the node.
		 */
		bool getConstraintAwarePositionIK(kinematics_msgs::GetConstraintAwarePositionIK::Request &request,
											kinematics_msgs::GetConstraintAwarePositionIK::Response &response);
	
	
	private: // methods
		/**
		 *
		 */
		bool setupCollisionEnvironment(void);
		
		/**
		 * Make the constraint aware IK service available to ROS components.
		 */
		void advertiseIK();
		
		/**
		 *
		 */
		bool isReady(motion_planning_msgs::ArmNavigationErrorCodes &error_code);
		
		/**
		 * @brief This is a callback function that gets called by the planning environment when a collision is found
		 * 
		 * The ccost and display arguments should be bound by the caller.
		 */
		void contactFound(collision_space::EnvironmentModel::Contact &contact);
		
		/**
		 *
		 */
		void initialPoseCheck(const KDL::JntArray &jnt_array,
								const KDL::Frame &p_in,
								motion_planning_msgs::ArmNavigationErrorCodes &error_code);
		
		/**
		 *
		 */
		void collisionCheck(const KDL::JntArray &jnt_array,
							const KDL::Frame &p_in,
							motion_planning_msgs::ArmNavigationErrorCodes &error_code);
		
		/**
		 *
		 */
		void printStringVec(const std::string &prefix, const std::vector<std::string> &string_vector);
	
	
	private: // variables and constants
		boost::shared_ptr<CobArmIKSolver> cob_arm_ik_solver_;
		
		// copied from pr2_arm_kinematics
		ros::NodeHandle node_handle_;
		ros::NodeHandle root_handle_;
		int dimension_;
		std::string root_name_;
		tf::TransformListener tf_;
		kinematics_msgs::KinematicSolverInfo ik_solver_info_;
		int free_angle_;
		double search_discretization_;




		ros::ServiceServer ik_collision_service_;
		ros::Publisher vis_marker_publisher_;
		ros::Publisher display_trajectory_publisher_;
		
		kinematics_msgs::PositionIKRequest ik_request_;
		std::string group_;
		bool use_collision_map_;
		bool visualize_solution_;
		bool setup_collision_environment_;
		std::vector<std::string> default_collision_links_;
		std::vector<std::string> end_effector_collision_links_;
		std::vector<motion_planning_msgs::LinkPadding> link_padding_;
		std::vector<motion_planning_msgs::AllowedContactSpecification> allowed_contacts_;
		
		planning_environment::CollisionModels *collision_models_;
		planning_environment::PlanningMonitor *planning_monitor_;
		motion_planning_msgs::OrderedCollisionOperations collision_operations_;
		motion_planning_msgs::Constraints constraints_;
		
		/**
		 * The name under which this node advertises its service.
		 */
		static const std::string IK_WITH_COLLISION_SERVICE;
		
		/**
		 * The timeout for ...
		 */
		static const double IK_DEFAULT_TIMEOUT;
};

#endif // COB_ARM_IK_CONSTRAINT_AWARE_H

