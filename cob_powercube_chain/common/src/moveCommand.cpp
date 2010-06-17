/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_powercube_chain
 * Description: Function implementation for motion command classes.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Felix Geibel, email:Felix.Geibel@gmx.de
 * Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 *
 * Date of creation: Sept 2007
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <powercube_chain/moveCommand.h>
#include <math.h>
#include <stddef.h>

RampCommand::RampCommand(double x0, double v0, double xtarget, double amax, double vmax)
	: moveCommand(),
		m_x0(x0),
		m_v0(v0), 
		m_xtarget(xtarget), 
		m_amax(amax), 
		m_vmax(vmax) 
{
	m_umkehr = false;
	m_nachumkehr = NULL;
	// Nur Beträge sollen verwendet werden, Richtung wird unten anhand von Delta gesetzt.
	m_vmax = fabs(m_vmax);
	m_amax = fabs(m_amax);
	// Set up the movement phases:
	double delta = m_xtarget - m_x0;
	if (delta < 0)
	{
		m_vmax = -m_vmax;
		m_amax = -m_amax;
	}
		
	if (m_v0 * m_vmax >= 0) // Momentangeschw. gleiches Vorzeichen wie Sollgeschw. (keine Umkehr nötig)
	{
		if (fabs(m_vmax) >= fabs(m_v0)) // Momentangeschw. ist vom Betrag kleiner als Sollgeschw.
		{
			// Kritische deltas bestimmen:
			double delta1 = (m_vmax*m_vmax - 0.5 * m_v0*m_v0) / m_amax;
			double delta2 = 0.5 * m_v0*m_v0 / m_amax;
			
			if (fabs(delta) >= fabs(delta1)) {
				m_T1 = (m_vmax-m_v0) / m_amax;
				m_a1 = m_amax;
				m_T3 = m_vmax / m_amax;
				m_a3 = -m_amax;
				m_T2 = (delta - delta1) / m_vmax;
				m_v2 = m_vmax;
			} 
			else if (fabs(delta) >= fabs(delta2)) {
				m_v2 = sqrt( delta*m_amax + 0.5*m_v0*m_v0 );
				m_v2 = (delta < 0)?-m_v2:m_v2;
				m_T1 = (m_v2-m_v0) / m_amax;
				m_a1 = m_amax;
				m_T2 = 0.0;
				m_T3 = m_v2 / m_amax;
				m_a3 = -m_amax;
			}
			else if (fabs(delta) > 0)
			{
				m_v2 = -sqrt( 0.5*m_v0*m_v0 - m_amax*delta );
				m_v2 = (delta < 0)?-m_v2:m_v2;
				m_T2 = 0.0;
				m_T1 = (m_v0 - m_v2) / m_amax;
				m_a1 = -m_amax;
				m_T3 = - m_v2 / m_amax;
				m_a3 = m_amax;
			}
			else // delta = 0
			{
				m_T1 = 0;
				m_T2 = 0;
				m_T3 = 0;
				m_a1 = 0;
				m_v2 = 0;
				m_a3 = 0;
			}
		} else // Momentangeschw. ist vom Betrag größer als Sollgeschw.
		{
			// Kritische Größen bestimmen:
			double delta1 = 0.5 * m_v0*m_v0 / m_amax;
			double vstern = m_v0 * 0.707106781; // v0 / wurzel(2)
			
			if (fabs(delta) >= fabs(delta1))
			{
				m_T1 = (m_v0-m_vmax) / m_amax;
				m_a1 = -m_amax;
				m_T3 = m_vmax / m_amax;
				m_a3 = -m_amax;
				m_T2 = (delta - delta1) / m_vmax;
				m_v2 = m_vmax;
			}
			else if (fabs(m_vmax) >= fabs(vstern))
			{
				m_v2 = -sqrt( 0.5*m_v0*m_v0 - m_amax*delta );
				m_v2 = (delta < 0)?-m_v2:m_v2;
				m_T2 = 0.0;
				m_T1 = (m_v0 - m_v2) / m_amax;
				m_a1 = -m_amax;
				m_T3 = - m_v2 / m_amax;
				m_a3 = m_amax;
			}					
			else // (fabs(m_vmax) < fabs(vstern))
			{
				m_T1 = (m_v0 + m_vmax) / m_amax;
				m_a1 = -m_amax;
				m_T2 = ( delta - (0.5*m_v0*m_v0/m_amax - m_vmax*m_vmax/m_amax) ) / m_vmax;
				m_v2 = -m_vmax;
				m_T3 = m_vmax / m_amax;
				m_a3 = m_amax;
			}
		}
	} // if (m_v0 * m_vmax >= 0)
	else // Momentangeschw. anderes Vorzeichen als Sollgeschw. -> Umkehr nötig
	{
		//std::cout << "Konstruktor\n";
		// Bis 0 fahren, danach neues RampCommand mit v0 = 0.
		m_a1 = m_amax;
		m_T1 = -m_v0 / m_amax;
		m_umkehr = true;
		// RampCommand::RampCommand(double x0, double v0, double xtarget, double amax, double vmax)
		m_nachumkehr = new RampCommand( m_x0 + 0.5*m_v0*m_T1, 0.0, m_xtarget, m_amax, m_vmax);
		//std::cout << "Zweites RampenKommando erzeugt\n";
		//std::cout << "Parameter: " << m_x0 + 0.5*m_v0*m_T1 << ", " <<  0.0 << ", " <<  m_xtarget << ", ";
		//std::cout << m_amax << ", " << m_vmax << "\n";
		m_T2 = 0;
		m_T3 = 0;
		m_v2 = 0;
		m_a3 = 0;
	}
	// Das wars Konstruktor fertig, Bewegungsablauf definiert.
}

RampCommand::RampCommand(const RampCommand& rc)
{
	// debug:
	//std::cout << "RampCommand: Copy Constructor called.\n";
	m_x0 = rc.m_x0;
	m_v0 = rc.m_v0;
	m_xtarget = rc.m_xtarget;
	m_amax = rc.m_amax;
	m_vmax = rc.m_vmax;
		
	m_T1 = rc.m_T1;
	m_T2 = rc.m_T2;
	m_T3 = rc.m_T3;
	m_a1 = rc.m_a1;
	m_v2 = rc.m_v2;
	m_a3 = rc.m_a3;
	
	m_umkehr = rc.m_umkehr;
	// Rekursionsgefahr! In dieser Implementierung wird sichergestellt, dass das m_nachumkehr RampCommand kein 
	// weiteres m_nachumkehr besitzt ( (*m_nachumkehr).m_nachumkehr=NULL ). Bitte bei evtl. Änderungen Sicherstellen, 
	// dass auch weiterhin keine unendliche Rekursion auftritt!
	if (rc.m_umkehr)
		m_nachumkehr = new RampCommand(*rc.m_nachumkehr);
	else
		m_nachumkehr = NULL;
}

RampCommand& RampCommand::operator=(const RampCommand& rc)
{
	// debug:
	//std::cout << "RampCommand: operator= called.\n";
	
	if (m_nachumkehr)
		delete m_nachumkehr;
	
	m_x0 = rc.m_x0;
	m_v0 = rc.m_v0;
	m_xtarget = rc.m_xtarget;
	m_amax = rc.m_amax;
	m_vmax = rc.m_vmax;
		
	m_T1 = rc.m_T1;
	m_T2 = rc.m_T2;
	m_T3 = rc.m_T3;
	m_a1 = rc.m_a1;
	m_v2 = rc.m_v2;
	m_a3 = rc.m_a3;
	
	m_umkehr = rc.m_umkehr;
	// Rekursionsgefahr! In dieser Implementierung wird sichergestellt, dass das m_nachumkehr RampCommand kein 
	// weiteres m_nachumkehr besitzt ( (*m_nachumkehr).m_nachumkehr=NULL ). Bitte bei evtl. Änderungen Sicherstellen, 
	// dass auch weiterhin keine unendliche Rekursion auftritt!
	if (rc.m_umkehr)
		m_nachumkehr = new RampCommand(*rc.m_nachumkehr);
	else
		m_nachumkehr = NULL;
	
	return *this;
}

		
/// @brief returns the planned position for TimeElapsed (seconds)
double RampCommand::getPos(double TimeElapsed)
{
	if (m_umkehr)
	{
		if (TimeElapsed <= m_T1) 
		{
			return m_x0 + m_v0 * TimeElapsed + 0.5 * m_a1 * TimeElapsed * TimeElapsed; // x = x0 + v0t + a/2*t^2
		} else {
			//std::cout << "in getPos(double)\n";
			return m_nachumkehr->getPos(TimeElapsed-m_T1);
		}
	} else
	{
		if (TimeElapsed <= m_T1) 
		{
			return m_x0 + m_v0 * TimeElapsed + 0.5 * m_a1 * TimeElapsed * TimeElapsed; // x = x0 + v0t + a/2*t^2
		} 
		else if (TimeElapsed <= m_T1 + m_T2)
		{
			return m_x0 + m_v0*m_T1 + 0.5*m_a1*m_T1*m_T1 + m_v2 * (TimeElapsed-m_T1); // x = x1 + v2*t
		}
		else if (TimeElapsed <= m_T1 + m_T2 + m_T3)
		{
			return m_x0 + m_v0*m_T1 + 0.5*m_a1*m_T1*m_T1 + m_v2*m_T2 + 
					m_v2*(TimeElapsed-m_T1-m_T2) + 0.5*m_a3*(TimeElapsed-m_T1-m_T2)*(TimeElapsed-m_T1-m_T2);
			// x = x2 + v2t + a/2*t^2
		}
		else // Bewegung Vorbei, Zielposition erreicht:
			return m_xtarget;
	}
}

/// @brief returns the planned velocity for TimeElapsed
double RampCommand::getVel(double TimeElapsed)
{
	if (m_umkehr)
	{
		if (TimeElapsed <= m_T1) 
		{
			return m_v0 + m_a1 * TimeElapsed;
		} else {
			return m_nachumkehr->getVel(TimeElapsed-m_T1);
		}
	} else
	{
		if (TimeElapsed <= m_T1) 
		{
			return m_v0 + m_a1 * TimeElapsed;
		} 
		else if (TimeElapsed <= m_T1 + m_T2)
		{
			return m_v2;
		}
		else if (TimeElapsed <= m_T1 + m_T2 + m_T3)
		{
			return m_v2 + m_a3 * (TimeElapsed - m_T1 - m_T2);
		}
		else // Bewegung Vorbei, Motor steht still:
			return 0.0;
	}
}

/// @brief returns the planned total time for the movement in seconds
double RampCommand::getTotalTime()
{
	//std::cout << "getTotalTime()\n";
	if (m_umkehr)
		return m_T1 + m_nachumkehr->getTotalTime();
	else
		return m_T1 + m_T2 + m_T3;
}

/// @brief Calculate the necessary a and v of a rampmove, so that the move will take the desired time
///  If possible, the deceleration phase should take the time T3, so that it can be made equal for all joints
void RampCommand::calculateAV(double x0, double v0, double xtarget, double time, double T3, double amax, double vmax, double& acc, double& vel)
{
	//std::ostream& out(debug);
	
	//out << "RampCommand::calculateAV(" << x0 << ", " << v0 << ", " << xtarget << ", " << time << ", " << T3 << ", " << amax << ", acc, vel)\n";
	double TG = time;
	
	if (TG <= 0)
	{
		//out << "Error in RampCommand::CalculateAV! total time is not greater than zero!\n";
		acc = 0;
		vel = 0;
	} else
	{
		double delta = xtarget - x0;
		//Vorzeichen nicht trauen, anhand von delta richtig setzen:
		amax = fabs(amax);
		amax = (delta>=0)?amax:-amax;
		
		double d1 = fabs(v0)*TG - 0.5 * T3 * fabs(v0);
		double d2h = (v0>0)?-T3*v0 + v0*sqrt(2*T3*TG):-T3*v0 - v0*sqrt(2*T3*TG);
		double d2l = (v0>0)?-T3*v0 - v0*sqrt(2*T3*TG):-T3*v0 + v0*sqrt(2*T3*TG);
		//double d2 = 0.5 * TG * v0;
		double d3 = 0.5 * v0 * v0 / amax;
		//out << "d1:\t" << d1 << "\n";
		//out << "d2h:\t" << d2h << "\n";
		//out << "d2l:\t" << d2l << "\n";
		//out << "d3:\t" << d3 << "\n";
		//out << "delta:\t" << delta << "\n";
		
		if (T3 > 0)
		{
			////////////////////////////////////////////////////////////////////////////////////
			//				Noch Fehlerhaft! Wurzelterme < 0 etc, richtig machen!			  //
			////////////////////////////////////////////////////////////////////////////////////
			if (fabs(delta) >= d1)
			{
				/* out << "  Fall 1\n"; */
				/* 	     ----------		*/
				/*      /          \	*/
				/*                  \	*/
				
				// v nach Rechnung Formel (I). a,b,c für Mitternachtsformel:
				double a = (TG / T3 - 1.0);
				double b = v0 - delta/T3;
				double c = - 0.5 * v0 * v0;
				//out << "  a=" << a << " | b=" << b << " | c=" << c << endl;
				// Mitternachtsformel:
				if (delta >= 0)
					vel = (-b + sqrt(b*b - 4.0*a*c)) / (2.0 * a);
				else 
					vel = (-b - sqrt(b*b - 4.0*a*c)) / (2.0 * a);
						
				// Jetzt a nach Formel (1):
				acc = vel / T3;
			} 
			else if (delta >= d2h || delta <= d2l)
			// fabs(delta) > d2s verhindert, dass Wurzelterm negativ wird!
			{
				/* out << "  Fall 2\n";	*/
				/* 	    \				*/
				/*       ----------		*/
				/*                 \	*/
				
				// v nach Rechnung Formel (II). a,b,c für Mitternachtsformel:
				double a = TG;
				double b = - delta - T3*v0;
				double c = 0.5 * v0 * v0 * T3;
				//out << "  a=" << a << " | b=" << b << " | c=" << c << endl;
				// Mitternachtsformel:
				if (delta >= 0)
					vel = (-b + sqrt(b*b - 4.0*a*c)) / (2.0 * a);
				else 
					vel = (-b - sqrt(b*b - 4.0*a*c)) / (2.0 * a);
						
				// Jetzt a nach Formel (1):
				acc = vel / T3;
			}
			else if (fabs(delta) >= d3)
			{
				// Dieser Fall bei bestimmten (relativ biedrigen) Deltas
				// in Kombination mit relativ hohen v0s auf, und für diesen
				// Fall kann ich keine Formel finden, für die Abbremsphase = T3 !!!
				// Ich finde es komisch, dass hier die Formel oben versagt (Wurzelterm < 0)
				// und die untere falsche Ergebnisse liefert...
				// Lösung, bei der Abbremsphase != T3: 
				//out << "  Fall 3:\n";
				
				acc = amax;
				
				if (delta*v0 > 0)
					vel = (2*acc*delta - v0*v0) / (2*acc*TG - 2*v0);
				else
					if (-4*acc*delta + acc*acc*TG*TG + 2*acc*TG*v0 - v0*v0 >= 0)
						if (delta>0)
						{
							// delta>0 (-> a>0), v0 < 0:
							vel = 0.5 * (acc*TG+v0 - sqrt(-4*acc*delta + acc*acc*TG*TG + 2*acc*TG*v0 - v0*v0));
						} else
						{
							// delta<0 (-> a<0), v0 > 0:
							vel = 0.5 * (acc*TG+v0 + sqrt(-4*acc*delta + acc*acc*TG*TG + 2*acc*TG*v0 - v0*v0));
						}
					else
					{
						// Dieser Spezialfall ist noch nicht gelöst, tritt bei eher größeren v0 auf
						// "Falsche Lösung" (Joint kommt zu früh an):
						vel = vmax;
					}
					
			}
			else
			{
				// Übersteuern, hier kann nicht gewährleistet werden, dass Abbremsphase
				// Während T3 (Gleichungssystem wäre überbestimmt).
				//out << "  Fall 4\n";
				if (4*delta*delta - 4*delta*TG*v0 + 2*TG*TG*v0*v0 >=0)
					// Wurzelterm macht keine Probleme
					if (delta*v0 > 0)
						acc = ( -2.0*delta + TG*v0 + sqrt(4*delta*delta - 4*delta*TG*v0 + 2*TG*TG*v0*v0) ) / (TG*TG);
					else
						acc = ( -2.0*delta + TG*v0 - sqrt(4*delta*delta - 4*delta*TG*v0 + 2*TG*TG*v0*v0) ) / (TG*TG);
				else
					// Wurzelterm macht Probleme, provisorische Lösung:
					acc = amax;
				
				vel = vmax;
			}
		} 
		else if (T3 == 0)
		{
			//out << "  Fall 5\n";
			// This is not "clean" yet ...
			// provisorische, inkorrekte Lösung
			acc = amax;
			vel = vmax;
		} else
		{
			//out << "  Error in RampCommand::CalculateAV! time T3 is negative!\n";
			acc = 0;
			vel = 0;
		}
	}
	//out << "  acc: " << acc << ",\tvel: " << vel << "\n";
	//out << "RampCommand::calculateAV ENDE\n";
	//debug.flush();
}

/*
CartesianRamp::CartesianRamp(AbsPos start, AbsPos ziel, double a, double v)
{
	m_x0 = start;
	m_xtarget = ziel;
	m_richtung = ziel - start;
	m_amax = a;
	m_vmax = v;
	if (m_richtung.getPosLength() > v*v/a)
	{
		// Phase mit konst. Geschw. wird erreicht:

		m_T1 = m_T3 = v / a;
		m_T2 = (m_richtung.getPosLength() - v*v/a) / v;

		// Wegparameter s immer positiv, deswegen keine weitere Fallunterscheidung nötig:
		m_a1 = a / m_richtung.getPosLength();
		m_v2 = v / m_richtung.getPosLength();
		m_a3 = -a / m_richtung.getPosLength();
	}
	else {
		// Phase konst. Geschw. wird nicht erreicht:

		m_a1 = a / m_richtung.getPosLength();
		m_v2 = sqrt(a / m_richtung.getPosLength());
		m_a3 = -a / m_richtung.getPosLength();

		m_T2 = 0.0;
		m_T1 = m_T3 = 1.0 / sqrt(m_a1);
	}
	// Bewegung vollständig charakterisiert.
}


/// @brief returns the planned position for TimeElapsed (seconds)
AbsPos CartesianRamp::getPos(double TimeElapsed) const
{
	if (TimeElapsed >= m_T1 + m_T2 + m_T3)
		return m_xtarget;
	else if (TimeElapsed >= m_T1 + m_T2)
	{
		return m_x0 + m_richtung * ( m_a1*m_T1*m_T1 + m_v2*m_T2 + m_a3*sqr(TimeElapsed-m_T1-m_T2) );
	}
	else if (TimeElapsed >= m_T1)
	{
		return m_x0 + m_richtung * ( m_a1*m_T1*m_T1 + m_v2*(TimeElapsed-m_T1) );
	}
	else if (TimeElapsed > 0.0)
	{
		return m_x0 + m_richtung * (m_a1*TimeElapsed*TimeElapsed);
	}
	else return m_x0;
}


/// @brief returns the planned velocity for TimeElapsed (seconds)
AbsPos CartesianRamp::getVel(double TimeElapsed) const
{
	if (TimeElapsed >= m_T1 + m_T2 + m_T3)
		return m_richtung * 0.0;
	else if (TimeElapsed >= m_T1 + m_T2)
	{
		return m_richtung * (m_v2 + m_a3 * (TimeElapsed-m_T1-m_T2));
	}
	else if (TimeElapsed >= m_T1)
	{
		return m_richtung * m_v2;
	}
	else if (TimeElapsed > 0.0)
	{
		return m_richtung * (m_a1*TimeElapsed);
	}
	else return m_richtung * 0.0;
}
*/
