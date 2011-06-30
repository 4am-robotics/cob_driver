
#include "ros/ros.h"
#include <kinematics_base/kinematics_base.h>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>
#include <map>
#include <iostream>
#define IKFAST_NO_MAIN
#ifdef IKFAST_REAL
typedef IKFAST_REAL IKReal;
#else
typedef double IKReal;
#endif


namespace cob3_arm_kinematics
{ 

class ik_solver_base{
public:
    virtual int solve(KDL::Frame &pose_frame, const std::vector<double> &ik_seed_state) = 0;
    virtual void getSolution(int i, std::vector<double> &solution) = 0;
    virtual void getClosestSolution(const std::vector<double> &ik_seed_state, std::vector<double> &solution) = 0;
};

template <class  T> class ikfast_solver: public ik_solver_base{
public:
    typedef bool (*ik_type)(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, std::vector<T>& vsolutions);
    ikfast_solver(ik_type ik,int numJoints):ik(ik),numJoints(numJoints) {}
    virtual int solve(KDL::Frame &pose_frame, const std::vector<double> &vfree){
      
      solutions.clear();
      ik(pose_frame.p.data, pose_frame.M.data, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
      
      return solutions.size();
    }
    virtual void getSolution(int i, std::vector<double> &solution){
      solution.clear();
      std::vector<IKReal> vsolfree(solutions[i].GetFree().size());
      solution.clear();
      solution.resize(numJoints);
      solutions[i].GetSolution(&solution[0],vsolfree.size()>0?&vsolfree[0]:NULL);
      std::cout << "solution " << i << ":" ;
      for(int j=0;j<numJoints; ++j)
	  std::cout << " " << solution[j];
      std::cout << std::endl;
	  
      //ROS_ERROR("%f %d",solution[2],vsolfree.size());
    }
    virtual void getClosestSolution(const std::vector<double> &ik_seed_state, std::vector<double> &solution){
      double mindist = 0;
      int minindex = -1;
      std::vector<double> sol;
      for(size_t i=0;i<solutions.size();++i){
	  getSolution(i,sol);
	  double dist = 0;
	  for(int j=0;j< numJoints;++j){
	    double diff = ik_seed_state[j] - sol[j];
	    dist += diff*diff;
	  }
	  if(minindex == -1 || dist<mindist){
	    minindex = i;
	    mindist = dist;
	  }
      }
      if(minindex >= 0) getSolution(minindex,solution);
    }
    
    
    
private:
    ik_type ik;
    std::vector<T> solutions;
    int numJoints;
};


namespace cob3_2{
#include "ikfast_cob3_2.cpp"
}

class IKFastKinematicsPlugin : public kinematics::KinematicsBase
{
  std::vector<std::string> joints;
  std::vector<double> jointMin;
  std::vector<double> jointMax;
  std::vector<std::string> links;
  std::map<std::string,size_t> link2index;
  std::string tip_name, root_name;
  KDL::Chain kdl_chain;
  boost::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_pose_solver;
  ik_solver_base* ik_solver;
  size_t numJoints;
  std::vector<int> freeParams;
  
public:

    IKFastKinematicsPlugin():ik_solver(0) {}
    ~IKFastKinematicsPlugin(){ if(ik_solver) delete ik_solver;}

  void fillFreeParams(int count, int *array) { freeParams.clear(); for(int i=0; i<count;++i) freeParams.push_back(array[i]); }
  
  bool initialize(std::string name) {

      ros::NodeHandle node_handle("~/"+name);

      std::string robot;
      node_handle.param("robot",robot,std::string());
      
      if(robot == "cob3-2"){
	fillFreeParams(cob3_2::getNumFreeParameters(),cob3_2::getFreeParameters());
	numJoints = cob3_2::getNumJoints();
	ik_solver = new ikfast_solver<cob3_2::IKSolution>(cob3_2::ik, numJoints);
      }else{
	ROS_FATAL("Robot '%s' unknown!",robot.c_str());
	return false;
      }
      
      if(freeParams.size()>1){
	ROS_FATAL("Only one free joint paramter supported!");
	return false;
      }
      
      
      urdf::Model robot_model;
      std::string xml_string;

      std::string urdf_xml,full_urdf_xml;
      node_handle.param("urdf_xml",urdf_xml,std::string("robot_description"));
      node_handle.searchParam(urdf_xml,full_urdf_xml);
      
      ROS_DEBUG("Reading xml file from parameter server\n");
      if (!node_handle.getParam(full_urdf_xml, xml_string))
      {
	ROS_FATAL("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
	return false;
      }

      if (!node_handle.getParam("root_name", root_name)){
	ROS_FATAL("PR2IK: No root name found on parameter server");
	return false;
      }
      if (!node_handle.getParam("tip_name", tip_name)){
	ROS_FATAL("PR2IK: No tip name found on parameter server");
	return false;
      }
      
      KDL::Tree tree;
      if (!kdl_parser::treeFromString(xml_string, tree))
      {
	ROS_ERROR("Could not initialize tree object");
	return false;
      }
      
/*      const KDL::SegmentMap & segments = tree.getSegments ();
      for( KDL::SegmentMap::const_iterator it =segments.begin();it!=segments.end();++it){
	ROS_ERROR("Segment %s",it->second.segment.getName().c_str());
      }*/
      
      if (!tree.getChain(root_name, tip_name, kdl_chain))
      {
	ROS_ERROR("Could not initialize kdl_chain object");
	return false;
      }
      
      int i=0; // segment number
      while(i < (int)kdl_chain.getNrOfSegments())
      {
	link2index.insert(std::make_pair(kdl_chain.getSegment(i).getName(),i));
	links.push_back(kdl_chain.getSegment(i).getName());
	if(links.size()>1){
	  joints.push_back(kdl_chain.getSegment(i).getJoint().getName());
	  ROS_ERROR("Joint %s",joints.back().c_str());

	}
	i++;
      }
      if(joints.size() != numJoints){
	ROS_ERROR("Joints count mismatch");
	return false;
      }
      
      jnt_to_pose_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
      
	
     return true;
    }

    bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                       const std::vector<double> &ik_seed_state,
                       std::vector<double> &solution,
		       int &error_code) {
	ROS_ERROR("getPositionIK");
	
	std::vector<double> vfree(freeParams.size());
	for(std::size_t i = 0; i < freeParams.size(); ++i){
	    int p = freeParams[i];
//	    ROS_ERROR("%u is %f",p,ik_seed_state[p]);
	    vfree[i] = ik_seed_state[p];
	}

	KDL::Frame frame;
	tf::PoseMsgToKDL(ik_pose,frame);

	int numsol = ik_solver->solve(frame,vfree);

		
	if(numsol){
	  ik_solver->getClosestSolution(ik_seed_state,solution);
	  ROS_ERROR("Solved!");
	  error_code = kinematics::SUCCESS;
	  return true;
	}
	
	error_code = kinematics::NO_IK_SOLUTION; 
	return false;
    }
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          const double &timeout,
                          std::vector<double> &solution,
			  int &error_code) {
  
	ROS_ERROR("searchPositionIK1");
	if(freeParams.size()==0){
	  return getPositionIK(ik_pose, ik_seed_state,solution, error_code);
	}
	
	KDL::Frame frame;
	tf::PoseMsgToKDL(ik_pose,frame);

	std::vector<double> vfree(freeParams.size());

	ros::Time maxTime = ros::Time::now() + ros::Duration(timeout);
	const double increment = 0.01; //TODO: make it a plugin parameter
	int counter = 0;

	while (ros::Time::now() < maxTime){
	    vfree[0] = ik_seed_state[freeParams[0]] + increment*counter;

	    int numsol = ik_solver->solve(frame,vfree);
	    
	    if(numsol > 0){
		ik_solver->getClosestSolution(ik_seed_state,solution);
		ROS_ERROR("Solved!");
		error_code = kinematics::SUCCESS;
		return true;
	    }
	    int i = 0;
	    do{
	      if( i >= 2){
		error_code = kinematics::NO_IK_SOLUTION; 
		return false;
	      }
	      counter = counter > 0? - counter: counter + 1;
	      ++i;
	    }while( counter *  increment < jointMin[freeParams[0]] ||  counter *  increment > jointMax[freeParams[0]] );
	     
	}
	error_code = kinematics::NO_IK_SOLUTION; 
 	return false;
    }      
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          const double &timeout,
                          std::vector<double> &solution,
                          const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &desired_pose_callback,
                          const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &solution_callback,
			  int &error_code){
      
	ROS_ERROR("searchPositionIK2");
	if(freeParams.size()==0){ // TODO: not closest, but valid solution!
	  return getPositionIK(ik_pose, ik_seed_state,solution, error_code);
	}
	
	if(!desired_pose_callback.empty())
	  desired_pose_callback(ik_pose,ik_seed_state,error_code);
	if(error_code < 0)
	{
	  ROS_ERROR("Could not find inverse kinematics for desired end-effector pose since the pose may be in collision");
	  return false;
	}

	KDL::Frame frame;
	tf::PoseMsgToKDL(ik_pose,frame);

	std::vector<double> vfree(freeParams.size());

	ros::Time maxTime = ros::Time::now() + ros::Duration(timeout);
	const double increment = 0.01; //TODO: make it a plugin parameter
	int counter = 0;

	std::vector<double> sol;
	while (ros::Time::now() < maxTime){
	    if(freeParams.size())
		vfree[0] = ik_seed_state[freeParams[0]] + increment*counter;

	    int numsol = ik_solver->solve(frame,vfree);
	    
	    if(numsol > 0){
		if(solution_callback.empty()){
		    ik_solver->getClosestSolution(ik_seed_state,solution);
		    error_code = kinematics::SUCCESS;
		    return true;
		}

		for (int s = 0; s < numsol; ++s){
		    ik_solver->getSolution(s,sol);
		    solution_callback(ik_pose,sol,error_code);
		    
		    if(error_code == kinematics::SUCCESS){
		      solution = sol;
		      ROS_ERROR("Solved!");
		      return true;
		    }
		}
	    }
	    int i = 0;
	    do{
	      if( i >= 2 || freeParams.size()==0){
		error_code = kinematics::NO_IK_SOLUTION; 
		return false;
	      }
	      counter = counter > 0? - counter: counter + 1 ;
	      ++i;
	    }while( counter *  increment < jointMin[freeParams[0]] ||  counter *  increment > jointMax[freeParams[0]] );
	     
	}
	error_code = kinematics::NO_IK_SOLUTION; 
 	return false;
   }      
    bool getPositionFK(const std::vector<std::string> &link_names,
                       const std::vector<double> &joint_angles, 
                       std::vector<geometry_msgs::Pose> &poses){
	KDL::Frame p_out;
	KDL::JntArray jnt_pos_in;
	geometry_msgs::PoseStamped pose;
	tf::Stamped<tf::Pose> tf_pose;

	jnt_pos_in.resize(joints.size());
	for(size_t i=0; i < joints.size(); i++)
	{
	  jnt_pos_in(i) = joint_angles[i];
	}

	poses.resize(link_names.size());

	bool valid = true;
	for(unsigned int i=0; i < poses.size(); i++)
	{
	  if(jnt_to_pose_solver->JntToCart(jnt_pos_in,p_out,link2index[link_names[i]]) >=0)
	  {
	    tf::PoseKDLToMsg(p_out,poses[i]);
	  }
	  else
	  {
	    ROS_ERROR("Could not compute FK for %s",link_names[i].c_str());
	    valid = false;
	  }
	}
  ROS_ERROR("getPositionFK %s",valid?"succeded":"failed");
	
	return valid;
    }      
    std::string getBaseFrame()  { 	 ROS_ERROR("getBaseFrame");
return root_name; }
    std::string getToolFrame() { 	 ROS_ERROR("getToolFrame");
return tip_name; }
    std::vector<std::string> getJointNames() { 	 ROS_ERROR("getJointNames");
return joints; }
    std::vector<std::string> getLinkNames() { 	 ROS_ERROR("getLinkNames");
return links; }
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(cob3_arm_kinematics,IKFastKinematicsPlugin, cob3_arm_kinematics::IKFastKinematicsPlugin, kinematics::KinematicsBase)