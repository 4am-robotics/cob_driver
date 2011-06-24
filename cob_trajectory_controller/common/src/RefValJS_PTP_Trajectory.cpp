#include <cob_trajectory_controller/RefValJS_PTP_Trajectory.h>
#include <stdexcept>
#include <algorithm>
#include <vector>

//#define PTP_TRAJ_DEBUG

const double RefValJS_PTP_Trajectory::weigths[] = { 1.5, 1.5, 1.0, 1.0, 0.8, 0.8, 0.7 };


inline double sqr(double x)
{
	return x*x;
}

RefValJS_PTP_Trajectory::RefValJS_PTP_Trajectory(const trajectory_msgs::JointTrajectory& trajectory, double v_rad_s, double a_rad_s2, bool smooth)
{	
	m_trajectory = trajectory;
	m_length = 0;
	m_stepSize = 0.4;
	//m_stepSize = 0.0175; // 0.0175 rad = 1°
	m_length_parts.clear();
	m_s_parts.clear();
	
	if ( m_trajectory.points.size() < 1 )
	{
		throw std::runtime_error("Tried to create reference values for empty trajectory!");
	}
	
	std::vector<std::vector<double> > zwischenPunkte;
	
	if (smooth == false)
	{
		// Maß dafür, wo groß die Rundung wird
// 		double between_stepsize = 2.5; // 0.2 rad = 11.5°
		//uhr-messmerf:
		double between_stepsize = 0.1; //verkleinert wegen Exception (s.u.)
		for (unsigned int i=0; i < m_trajectory.points.size()-1; i++)
		{
			std::vector<double> direction;
			direction.resize( m_trajectory.points[i+1].positions.size());
			double len = 0;
			for(unsigned int j = 0; j <  m_trajectory.points[i+1].positions.size(); j++)
			{
				direction.at(j) = m_trajectory.points[i+1].positions.at(j)-m_trajectory.points[i].positions.at(j);
				len += direction.at(j) * direction.at(j);
			}
			double dist = sqrt(len);
			int num_segs = ceil(dist/between_stepsize);
			zwischenPunkte.resize(num_segs);
			for (int j=0; j < num_segs; j++)
			{
				std::vector<double> betw_joint;
				betw_joint.resize(direction.size());
				for(unsigned int d=0; d < direction.size(); d++)
				{
					betw_joint.at(d) = m_trajectory.points[i].positions.at(d) + direction.at(d) * ( (double)j / (double)num_segs );
				}
				zwischenPunkte.at(j) = betw_joint;
			}
		}
		zwischenPunkte.push_back( m_trajectory.points.back().positions );
	} else 
	{
		zwischenPunkte.resize(trajectory.points.size());
		for(unsigned int i = 0; i < trajectory.points.size(); i++)
		{
			zwischenPunkte.at(i) = trajectory.points.at(i).positions;
		}
	}
	ROS_INFO("Calculated %d zwischenPunkte", zwischenPunkte.size());
	
	m_TrajectorySpline.setCtrlPoints(zwischenPunkte);
	
	// Note: unlike the name of the function suggests, the distance between any two neigbouring points
	// is not always the same!
	if ( m_TrajectorySpline.ipoWithConstSampleDist(m_stepSize, m_SplinePoints)==false )
	{
		//uhr-messmerf: diese exception wird geworfen, wenn nicht genügend Stützpunkte vorhanden sind (kleiner m_iGrad=3)
		//uhr-messmerf: dies tritt auf, wenn die Punkte der Trajectory pfad zu nah beieinander liegen (dist < between_stepsize=2.5)
		throw std::runtime_error("Error in BSplineND::ipoWithConstSampleDist!");
	}
	m_SplinePoints = zwischenPunkte;
	ROS_INFO("Calculated %d splinepoints", m_SplinePoints.size());
	
	m_length_cumulated.clear();
	m_length_cumulated.push_back(0.0);
	
	for (unsigned int i=0; i < m_SplinePoints.size()-1; i++)
	{
		std::vector<double> dist;
		dist.resize(m_SplinePoints[i].size());
		for(unsigned int j=0; j < m_SplinePoints[i].size(); j++)
		{
			dist.at(j) = m_SplinePoints[i+1].at(j) - m_SplinePoints[i].at(j);
		}
		//double max = fabs(direction.getMax());
		//double min = fabs(direction.getMin());
		m_length_parts.push_back( norm(dist) );
		m_length += norm(dist);
		m_length_cumulated.push_back(m_length);
	}
	
	m_param_length = m_TrajectorySpline.getMaxdPos();

	for (unsigned int i=0; i < m_length_parts.size(); i++)
	{
		m_s_parts.push_back( m_length_parts[i] / m_length );
	}
	
	m_v_rad_s = v_rad_s;
	m_a_rad_s2 = a_rad_s2;
	
	/* Parameter für Beschl.begrenzte Fahrt: */	
	double a = fabs(m_a_rad_s2);
	double v = fabs(m_v_rad_s);
	
	if (m_length > v*v/a)
	{
		// Phase mit konst. Geschw. wird erreicht:
		m_T1 = m_T3 = v / a;
		m_T2 = (m_length - v*v/a) / v;
		
		// Wegparameter s immer positiv, deswegen keine weitere Fallunterscheidung nötig:
		m_sv2 = 1.0 / (m_T1 + m_T2);
		m_sa1 = m_sv2 / m_T1;
		m_sa3 = -m_sa1;
	} 
	else {
		// Phase konst. Geschw. wird nicht erreicht:
		m_T2 = 0.0;
		m_T1 = m_T3 = sqrt(m_length / a);

		m_sv2 = 1.0 / m_T1;
		m_sa1 = m_sv2 / m_T1;
		m_sa3 = -m_sa1;
		
	}
	// Bewegung vollständig charakterisiert.
		
}

std::vector<double> RefValJS_PTP_Trajectory::r(double s) const
{
	//printw("Line %d\ts: %f\n", __LINE__, s);
	std::vector<double> soll;
	if (s <= 0)
	{
		soll = m_trajectory.points.front().positions;
	} else
	if (s < 1)
	{
		// since the Distances between the points of m_SplinePoints are not equal,
		// we first need to find the last i where m_length_cumulated[i] is smaller than s*m_length
		
		// return the first index i, where m_length_cumulated[i] is not smaller s * m_length
		// more information under: http://www.cplusplus.com/reference/algorithm/lower_bound/
		//typedef std::vector<double> dvec;
		
		
		vecd_it start = m_length_cumulated.begin();
		vecd_it end = m_length_cumulated.end();
		vecd_it it = upper_bound(start, end, s * m_length);
		
		int i = int(it-start) - 1;
		double frac = (s * m_length - m_length_cumulated[i]) / m_length_parts[i];
		

		/*
		int i = floor( s * m_param_length / m_stepSize);
		double frac = s * m_param_length / m_stepSize - (double) i;
		*/
		// interpolate
		soll.resize(m_SplinePoints[i].size());
		for(unsigned int j = 0; j < m_SplinePoints[i].size(); j++)
		{
			soll.at(j) = m_SplinePoints[i].at(j) + (m_SplinePoints[i+1].at(j)-m_SplinePoints[i].at(j))*frac;
		}
	}
	else
	{
		soll = m_trajectory.points.back().positions;
	}
	return soll;
}

double RefValJS_PTP_Trajectory::s(double t) const
{
	if (t >= m_T1 + m_T2 + m_T3)
		return 1.0;
	else if (t >= m_T1 + m_T2)
	{
		return 0.5*m_sa1*m_T1*m_T1 + m_sv2*m_T2 + m_sv2*(t-m_T1-m_T2) + 0.5*m_sa3*sqr(t-m_T1-m_T2);
	} 
	else if (t >= m_T1)
	{
		return 0.5*m_sa1*m_T1*m_T1 + m_sv2*(t-m_T1);
	}
	else if (t > 0.0)
	{
		return 0.5*m_sa1*t*t;
	}
	else return 0.0;
}

double RefValJS_PTP_Trajectory::ds_dt(double t) const
{
	if (t >= m_T1 + m_T2 + m_T3)
		return 0.0;
	else if (t >= m_T1 + m_T2)
	{
		return m_sv2 + m_sa3*(t-m_T1-m_T2);
	} 
	else if (t >= m_T1)
	{
		return m_sv2;
	}
	else if (t > 0.0)
	{
		return m_sa1*t;
	}
	else return 0.0;
}


std::vector<double> RefValJS_PTP_Trajectory::dr_ds(double s) const
{
	//printw("Line %d\ts: %f\n", __LINE__, s);
	std::vector<double> result = m_trajectory.points.front().positions;
	if (s < 0.0 || s >= 1.0)
	{
		for(unsigned int j=0; j < result.size(); j++)
			result.at(j) = 0.0;
	}
	else
	{
		
		vecd_it start = m_length_cumulated.begin();
		vecd_it end = m_length_cumulated.end();
		vecd_it it = upper_bound(start, end, s * m_length);
		
		int i = int(it-start) - 1;	
		double frac = (s * m_length - m_length_cumulated[i]) / m_length_parts[i];
		
		
		/*
		int i = floor( s * m_param_length / m_stepSize);
		double frac = s * m_param_length / m_stepSize - (double) i;
		*/
		
		std::vector<double> vi;	// vi
		std::vector<double> vii;// vi+1
		
		double step_s = m_stepSize / m_param_length;

		if ( i == 0 )
		{
			// vi rechtsseitig approximieren
			step_s = m_length_parts[0] / m_length;
			vi.resize(m_SplinePoints[i].size());
			for(unsigned int k = 0; k < m_SplinePoints[i].size(); k++)
			{
				vi.at(k) = (m_SplinePoints[1].at(k) - m_SplinePoints[0].at(k)) / step_s;
			}
			vii = vi;
			// vi+1 zentrisch approximieren
			//step_s = (m_length_parts[i]+m_length_parts[i+1]) / m_length;
			//vii = (m_SplinePoints[i+2] - m_SplinePoints[i]) / (2.0 * step_s);
		} else
		if ( i == (int) m_SplinePoints.size() - 2 )
		{
			// vi zentrisch:
			//step_s = (m_length_parts[i]+m_length_parts[i-1]) / m_length;
			//vi = (m_SplinePoints[i+1] - m_SplinePoints[i-1]) / (2.0 * step_s);
			// vi+1 linksseitig:
			step_s = (m_length_parts[i]) / m_length;
			vii.resize(m_SplinePoints[i].size());
			for(unsigned int k = 0; k < m_SplinePoints[i].size(); k++)
				vii.at(k) = (m_SplinePoints[i+1].at(k) - m_SplinePoints[i].at(k)) / step_s;
			vi = vii;
		} else
		{
			// beide zentrisch:
			step_s = (m_length_parts[i]+m_length_parts[i-1]) / m_length;
			vi.resize(m_SplinePoints[i].size());
			for(unsigned int k = 0; k < m_SplinePoints[i].size(); k++)
				vi.at(k) = (m_SplinePoints[i+1].at(k) - m_SplinePoints[i-1].at(k)) / step_s;
			step_s = (m_length_parts[i]+m_length_parts[i+1]) / m_length;
			vii.resize(m_SplinePoints[i].size());
			for(unsigned int k = 0; k < m_SplinePoints[i].size(); k++)
				vii.at(k) = (m_SplinePoints[i+2].at(k) - m_SplinePoints[i].at(k)) / step_s;
		}

		// linear interpolieren:
		result.resize(m_SplinePoints[i].size());
		for(unsigned int k = 0; k < m_SplinePoints[i].size(); k++)
			result.at(k) = vi.at(k) + (vii.at(k)-vi.at(k))*frac;
	}

	return result;
}




