/********************************************************************
 *                                                                  *
 *                   RefValJS_PTP_Trajectory                        *
 *                                                                  *
 ********************************************************************/
 
#ifndef _REFVALJS_PTP_TRAJECTORY_H_
#define _REFVALJS_PTP_TRAJECTORY_H_

#include <cob_trajectory_controller/RefVal_JS.h>
#include <vector>
#include <cob_trajectory_controller/BSplineND.h>
#include <trajectory_msgs/JointTrajectory.h>

class RefValJS_PTP_Trajectory : public RefVal_JS
{
	public:
		/* smooth = false:
			additional knots between the trajectory points will be generated to make the spline follow straight lines
			and just have the corners rounded (default)
		   smooth = true:
		   	no additional points will be generated, path will be lot smoother, but will have significant differences
		   	between a path were the trajectory points are connected by straight lines. Do not use in conjunction with
		   	a path planner!
		*/
		RefValJS_PTP_Trajectory(const trajectory_msgs::JointTrajectory& trajectory, double v_rad_s, double a_rad_s2, bool smooth=false);
		//RefValJS_PTP_Trajectory(const std::vector<Jointd>& trajectory, Jointd start, Jointd startvel, double v_rad_s, double a_rad_s2, bool smooth=false);
		
		std::vector<double> r(double s) const;
		double s(double t) const;

		std::vector<double> dr_ds(double s) const;
		double ds_dt(double t) const;
		
		double getTotalTime() const { return m_T1 + m_T2 + m_T3; }
		
		std::vector<double> getLengthParts() const { return m_length_parts; }
		
	protected:
		double norm(const std::vector<double>& j);
		double norm_max(const std::vector<double>& j);
		double norm_sqr(const std::vector<double>& j);
		double norm_weighted(const std::vector<double>& j);
		trajectory_msgs::JointTrajectory m_trajectory;
		
		typedef std::vector<double> vecd;
		typedef std::vector<double>::const_iterator vecd_it;
		vecd m_length_parts;
		vecd m_length_cumulated;
		vecd m_s_parts;
		
		BSplineND< std::vector<double> > m_TrajectorySpline;
		std::vector< std::vector<double> > m_SplinePoints;
		

		double m_stepSize;
		double m_length;
		double m_param_length;
		
		double m_v_rad_s;
		double m_a_rad_s2;
		
		double m_T1;	// Dauer der Phase konst. Beschl.
		double m_T2;	// Dauer der Phase konst. Geschw.
		double m_T3;	// Dauer der Phase konst. Verzög.
		
		double m_sa1;	// "Beschl." des Wegparameters s in Phase 1
		double m_sv2;	// "Geschw." des Wegparameters s in Phase 2
		double m_sa3;	// "Verzög." des Wegparameters s in Phase 3
		
		static const double weigths[];
		
};


inline double RefValJS_PTP_Trajectory::norm(const std::vector<double>& j)
{
	// which norm should be used?
	return norm_weighted(j);
}

inline double RefValJS_PTP_Trajectory::norm_max(const std::vector<double>& j)
{
	double max = j.at(0);
	for	(unsigned int i = 0; i<j.size(); i++)
	{
		if(j.at(i) < max)
			max = j.at(i);
	}
	return max;
}

inline double RefValJS_PTP_Trajectory::norm_sqr(const std::vector<double>& j)
{
	double l = 0;
	for (unsigned int i = 0; i < j.size(); i++)
	{
	        l += j[i] * j[i];
	}
	return sqrt(l);
}

inline double RefValJS_PTP_Trajectory::norm_weighted(const std::vector<double>& j)
{
	double l = 0;
	if ( j.size() == 7 )
	{
		for (unsigned int i = 0; i < j.size(); i++)
		{
			l += j[i]* weigths[i] * j[i] * weigths[i];
		}
	}
	else
	{
		for (unsigned int i = 0; i < j.size(); i++)
		{
			l += j[i] * j[i];
		}
	}
	return sqrt(l);
}



#endif

