#ifndef __GENERIC_ARM_CTRL_V4_H_
#define __GENERIC_ARM_CTRL_V4_H_

#include "ros/ros.h"
//#include "Manipulation/ManipUtil/Joint.h"
//#include "ManipUtil/datastructsManipulator.h"
//#include "ManipUtil/ManipulatorXML.h"
//#include "Interfaces/KinematicsInterface.h"
//#include "Interfaces/armInterface_ctrl.h"

#include <trajectory_msgs/JointTrajectory.h>

#include <cob_trajectory_controller/RefVal_JS.h>
#include <cob_trajectory_controller/TimeStamp.h>

class genericArmCtrl
{
	public: 
		
		genericArmCtrl(int DOF);
		~genericArmCtrl();
		
		std::vector<double> GetPTPvel() const;
		std::vector<double> GetPTPacc() const;
		void SetPTPvel(double vel);
		void SetPTPacc(double acc);
	
		// void stop(); --> TODO: better reset

		bool step(std::vector<double> current_pos, std::vector<double> & desired_vel);

		bool moveThetas(std::vector<double> conf_goal, std::vector<double> conf_current);
		bool moveTrajectory(trajectory_msgs::JointTrajectory pfad, std::vector<double> conf_current);


//		bool movePos(AbsPos position);

		int m_DOF;
		
		RefVal_JS* m_pRefVals;
		
		std::vector<double> m_vel_js;
		std::vector<double> m_acc_js;
		bool isMoving;

		TimeStamp startTime_;
		double TotalTime_;
		double m_P;
		double m_Vorsteuer;
		double m_AllowedError;
		double m_CurrentError;
		double m_TargetError;
		double m_ExtraTime;	// Zusätzliche Zeit, um evtl. verbleibende Regelfehler auszuregeln
};



#endif
