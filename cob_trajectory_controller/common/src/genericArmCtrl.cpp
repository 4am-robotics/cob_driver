#include <cob_trajectory_controller/genericArmCtrl.h>
#include <fstream>
#include <cob_trajectory_controller/RefValJS_PTP_Trajectory.h>
#include <cob_trajectory_controller/RefValJS_PTP.h>


/********************************************************************
 *          Some macros for error checking and handling             *
 ********************************************************************/



#define STD_CHECK_PROCEDURE() 											\
if ( isMoving )														\
{																		\
	ROS_WARN("Manipulator still in motion, call stop first!");		\
	return false;														\
}																		\
if (m_pRefVals != NULL)	{												\
	delete m_pRefVals;													\
	m_pRefVals = NULL;													\
}


#define START_CTRL_JOB(info)											\
if ( m_cThread.startJob(m_pRefVals) == false ) {						\
	m_ErrorMsg.assign("Internal Error: Unable to start movement, other control Job still active.");	\
	return false;														\
} else {																\s												\
	return true;														\
}




/********************************************************************
 *                  Initialisierungsfunktionen                      *
 ********************************************************************/


genericArmCtrl::genericArmCtrl(int DOF)
{
	
	m_DOF = DOF;
	
	m_pRefVals = NULL;
	
	isMoving = false;

	//TODO: make configurable
	SetPTPvel(0.7);
	SetPTPacc(0.2);

	//m_P = 2.5;
	m_P = 4.0;
	m_Vorsteuer = 0.9;
	m_AllowedError = 0.75;//0.5;//0.25; // rad
	m_CurrentError = 0.0; // rad
	m_TargetError = 0.02; // rad;

	m_ExtraTime = 3;	// s
	
}


genericArmCtrl::~genericArmCtrl()
{
	if (m_pRefVals != NULL)
		delete m_pRefVals;	
}

/********************************************************************
 *                      Parameter Abfragen                          *
 ********************************************************************/

std::vector<double> genericArmCtrl::GetPTPvel() const
	{ return m_vel_js; }

std::vector<double> genericArmCtrl::GetPTPacc() const
	{ return m_acc_js; }




/********************************************************************
 *                       Parameter Setzen                           *
 ********************************************************************/

void genericArmCtrl::SetPTPvel(double vel)
{
	m_vel_js.resize(m_DOF);
	for (int i = 0; i < m_DOF; i++)
		m_vel_js.at(i) = vel;
}

void genericArmCtrl::SetPTPacc(double acc)
{
	m_acc_js.resize(m_DOF);
	for (int i = 0; i < m_DOF; i++)
		m_acc_js.at(i) = acc;
}

/********************************************************************
 *                   PTP (Joint Space) Motion:                      *
 ********************************************************************/

/// @brief Will move the arm to a goal configuration in Joint Space
bool genericArmCtrl::moveThetas(std::vector<double> conf_goal, std::vector<double> conf_current)
{
	/* Prüfen ob Arm noch in Bewegung & Prüfen ob alte Sollwerte gelöscht werden müssen: */
	STD_CHECK_PROCEDURE()
	
	/* Sollwerte generieren: */
	double vel = 10000;
	for	(unsigned int i = 0; i<m_vel_js.size(); i++)
	{
		if(m_vel_js.at(i) < vel)
			vel = m_vel_js.at(i);
	}
	double acc = 10000;
	for	(unsigned int i = 0; i<m_acc_js.size(); i++)
	{
		if(m_acc_js.at(i) < acc)
			acc = m_acc_js.at(i);
	}

					;
	m_pRefVals = new RefValJS_PTP(conf_current, conf_goal, vel, acc);
	startTime_.SetNow();
	isMoving = true;
	TotalTime_ = m_pRefVals->getTotalTime();
	ROS_INFO("Starting control of trajectory: %f s long", TotalTime_);
	
}
	
bool genericArmCtrl::moveTrajectory(trajectory_msgs::JointTrajectory pfad, std::vector<double> conf_current)
{
	/* Prüfen, ob erster Punkt nah genug an momentaner Position ist: */
	if (pfad.points.size() == 2)
	{
		return moveThetas(pfad.points[1].positions, conf_current);
	}
	for(unsigned int i = 0; i < pfad.points.front().positions.size(); i++ )
	{
		if((pfad.points.front().positions.at(i) - conf_current.at(i)) > 0.15)
		{
			ROS_ERROR("Cannot start trajectory motion, manipulator not in trajectory start position.");
			return false;
		}
	}
	
	/* Prüfen ob Arm noch in Bewegung & Prüfen ob alte Sollwerte gelöscht werden müssen: */
	STD_CHECK_PROCEDURE()
	
	/* Sollwerte generieren: */
	double vel = m_vel_js.at(0);
	for	(unsigned int i = 0; i<m_vel_js.size(); i++)
	{
		if(m_vel_js.at(i) < vel)
			vel = m_vel_js.at(i);
	}
	double acc = m_acc_js.at(0);
	for	(unsigned int i = 0; i<m_acc_js.size(); i++)
	{
		if(m_acc_js.at(i) < acc)
			acc = m_acc_js.at(i);
	}
	m_pRefVals = new RefValJS_PTP_Trajectory(pfad, vel, acc, true);
	
	/* Regeljob starten: */
	startTime_.SetNow();
	isMoving = true;
	TotalTime_ = m_pRefVals->getTotalTime();
	ROS_INFO("Starting control of trajectory: %f s long", TotalTime_);

	//START_CTRL_JOB(Trajectory)
	return true;
}

//bool genericArmCtrl::movePos(AbsPos position)
//{
//	/* Prüfen ob Arm noch in Bewegung & Prüfen ob alte Sollwerte gelöscht werden müssen: */
//	STD_CHECK_PROCEDURE()
//
//	Jointd start = getCurrentAngles();
//
//	// attempt to move along straight line:
//	RefValJS_CartesianStraight* rv = new RefValJS_CartesianStraight(m_pKin, start);
//	m_pRefVals = rv;
//	if ( rv->calculate(position, m_vel_cartesian, m_acc_cartesian) == false )
//	{
//		m_ErrorMsg.assign("MovePos: Target Position not reachable.");
//		#ifdef GENERIC_ARM_CTRL_LOG
//		m_log << m_ErrorMsg << "\n";
//		#endif
//		return false;
//	}
//
//	/* Regeljob starten: */
//	START_CTRL_JOB(Cartesian)
//}

bool genericArmCtrl::step(std::vector<double> current_pos, std::vector<double> & desired_vel)
{
	if(isMoving)
	{
		TimeStamp timeNow_;
		timeNow_.SetNow();

		if ( m_pRefVals == NULL )
				return false;
		double t = timeNow_ - startTime_;
		std::vector<double> m_qsoll = m_pRefVals->r_t( t );
		std::vector<double> m_vsoll = m_pRefVals->dr_dt(t);

		double len = 0;
		for(unsigned int i = 0; i < m_DOF; i++)
		{
			 len +=  (m_qsoll.at(i) - current_pos.at(i)) * (m_qsoll.at(i) - current_pos.at(i));
		}
		m_CurrentError = sqrt(len);
		if ( m_pRefVals != NULL && t < TotalTime_ + m_ExtraTime && (m_CurrentError > m_TargetError || t < TotalTime_) )
		{
				if ( m_CurrentError >= m_AllowedError )
				{
					ROS_ERROR("Current control error exceeds limit: %f >= %f", m_CurrentError, m_AllowedError);
					ROS_ERROR("Current Soll: %f %f %f %f %f %f %f ", m_qsoll.at(0), m_qsoll.at(1), m_qsoll.at(2), m_qsoll.at(3), m_qsoll.at(4), m_qsoll.at(5), m_qsoll.at(6));
					ROS_ERROR("Current Ist: %f %f %f %f %f %f %f ", current_pos.at(0), current_pos.at(1), current_pos.at(2), current_pos.at(3), current_pos.at(4), current_pos.at(5), current_pos.at(6));
					isMoving = false;
					return false;
				}
				desired_vel.resize(m_DOF);
				/* Vorsteuerung + P-Lageregler: */
				for(int i = 0; i < m_DOF; i++)
				{
					desired_vel.at(i) = m_vsoll.at(i) * m_Vorsteuer + ( m_qsoll.at(i) - current_pos.at(i) ) * m_P;
				}

				return true;
		}
		else /* entweder sind keine Sollwerte vorhanden, oder Zeit ist abgelaufen */
		{
			ROS_INFO("Probably finished trajectory");
			isMoving = false;
			desired_vel.resize(m_DOF);
			for(unsigned int i = 0; i < m_DOF; i++)
			{
				desired_vel.at(i) = 0.0;
			}
			return true;
		}
	}
	return false;
}




