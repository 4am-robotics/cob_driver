#include <cob_trajectory_controller/RefValJS_PTP.h>

inline double sqr(double x)
{
	return x*x;
}

const double RefValJS_PTP::weigths[] = { 1.5, 1.5, 1.0, 1.0, 0.8, 0.8, 0.7 };


RefValJS_PTP::RefValJS_PTP(const std::vector<double>& start, const std::vector<double>& ziel, double v_rad_s, double a_rad_s2)
{	
	m_start = start;
	m_ziel = ziel;
	
	m_direction.resize(start.size());
	for(unsigned int i = 0; i < start.size(); i++)
		m_direction.at(i) = ziel.at(i) - start.at(i);
	double max = m_direction.at(0);
	double min = m_direction.at(0);
	for(unsigned int i = 0; i < m_direction.size(); i++)
	{
		if(max < m_direction.at(i))
			max = m_direction.at(i);
		if(min > m_direction.at(i))
			min = m_direction.at(i);
	}
	max = fabs(max);
	min = fabs(min);

	//m_length = (max>min)?max:min;
	m_length = norm(m_direction);
	
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

	
std::vector<double> RefValJS_PTP::r(double s) const
{
	std::vector<double> soll;
	soll.resize(m_start.size());
	if (s <= 0)
	{
		soll = m_start;
	} else
	if (s < 1)
	{
		for(unsigned int i = 0; i < m_start.size(); i++)
			soll.at(i) = m_start.at(i) + m_direction.at(i) * s;
	}
	else
	{
		soll = m_ziel;
	}
	return soll;
}

double RefValJS_PTP::s(double t) const
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

double RefValJS_PTP::ds_dt(double t) const
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

std::vector<double> RefValJS_PTP::dr_ds(double s) const
{
	std::vector<double> result = m_start;
	if (s < 0.0 || s >= 1.0)
	{
		for(unsigned int i = 0; i<result.size(); i++)
			result.at(i) = 0.0;
	}
	else
	{
		result = m_direction;
	}

	return result;
}


