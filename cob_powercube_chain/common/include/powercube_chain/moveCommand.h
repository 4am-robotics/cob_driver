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
 * ROS stack name: cob3_driver
 * ROS package name: powercube_chain
 * Description: These classes describe motion commands.
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

#ifndef __MOVE_COMMAND_H__
#define __MOVE_COMMAND_H__

#include <powercube_chain/datastructsManipulator.h>
#include <powercube_chain/TimeStamp.h>

class moveCommand
{
	public:
		moveCommand() {;}
		virtual ~moveCommand() {;}

		/////////////////////////////////////////////////////////////////////////
		//               Child Classes have to define these:                   //
		/////////////////////////////////////////////////////////////////////////

		/// @brief returns the planned position for TimeElapsed (seconds)
		virtual double getPos(double TimeElapsed)=0;

		/// @brief returns the planned velocity for TimeElapsed (seconds)
		virtual double getVel(double TimeElapsed)=0;

		/// @brief returns the planned total time for the movement (in seconds)
		virtual double getTotalTime()=0;

		/////////////////////////////////////////////////////////////////////////
		//                Functions useful for simulation:                     //
		//          start() has to be called before the other functions        //
		/////////////////////////////////////////////////////////////////////////

		/// @brief sets the starttime for the movement to the current time
		virtual void start() { m_timeStarted.SetNow(); }

		/// @brief returns true if the the end of the movement is not reached yet
		virtual bool isActive() { m_now.SetNow(); return ( m_now-m_timeStarted > getTotalTime() )?false:true; std::cerr << "========Total Time: " << getTotalTime() << "\n";}

		/// @brief returns the planned position at time of function call (Sollposition)
		virtual double pos() { m_now.SetNow(); return getPos( m_now-m_timeStarted ); }

		/// @brief returns the planned velocity at time of function call (Sollgeschwindigkeit)
		virtual double vel() { m_now.SetNow(); return getVel( m_now-m_timeStarted ); }

		/// @brief returns remaining time
		virtual double timeRemaining() { m_now.SetNow(); return (m_timeStarted - m_now) + getTotalTime(); }

	protected:
		TimeStamp m_timeStarted;
		TimeStamp m_now;
};



class RampCommand : public moveCommand
{
	public:
		RampCommand(double x0, double v0, double xtarget, double amax, double vmax);
		RampCommand(const RampCommand& rc);
		
		virtual RampCommand& operator=(const RampCommand& rc);
			  
		virtual ~RampCommand() { if (m_nachumkehr) delete m_nachumkehr; }
		
		/// @brief returns the planned position for TimeElapsed (seconds)
		virtual double getPos(double TimeElapsed);
		double getPos() { return moveCommand::pos(); }
		
		/// @brief returns the planned velocity for TimeElapsed (seconds)
		virtual double getVel(double TimeElapsed);
		double getVel() { return moveCommand::vel(); }
		
		/// @brief returns the planned total time for the movement (in seconds)
		virtual double getTotalTime();

		virtual bool inPhase1() { m_now.SetNow(); return ( m_now-m_timeStarted <= m_T1 )?true:false; }

		virtual bool inPhase3()
		{ m_now.SetNow(); return ( m_now-m_timeStarted > m_T1 + m_T2 && m_now-m_timeStarted < m_T1 + m_T2 + m_T3)?true:false; }

		/// @brief Return the times of the different phases of the ramp move
		virtual double T1() { return (m_umkehr)?(m_T1 + m_nachumkehr->T1()):m_T1; }
		virtual double T2() { return (m_umkehr)?m_nachumkehr->T2():m_T2; }
		virtual double T3() { return (m_umkehr)?m_nachumkehr->T3():m_T3; }
		
		/// @brief Calculate the necessary a and v of a rampmove, so that the move will take the desired time
		static void calculateAV(double x0, double v0, double xtarget, double time, double T3, double amax, double vmax, double& a, double& v);
		
	private:

		static std::ofstream debug;
		
		double m_x0, m_v0;
		double m_xtarget;
		double m_amax, m_vmax;
		
		double m_T1, m_T2, m_T3;
		double m_a1, m_v2, m_a3;
		bool m_umkehr;
		RampCommand * m_nachumkehr;
};

//std::ofstream RampCommand::debug("debugRampCommand.txt");

/*
class CartesianRamp
{
	public:
		// unit amax: mm/s^2, unit vmax: mm/s
		CartesianRamp(AbsPos start, AbsPos ziel, double amax, double vmax);
		virtual ~CartesianRamp() {;}


		/// @brief returns the planned position for TimeElapsed (seconds)
		virtual AbsPos getPos(double TimeElapsed) const;

		/// @brief returns the planned velocity for TimeElapsed (seconds)
		virtual AbsPos getVel(double TimeElapsed) const;

		/// @brief returns the planned total time for the movement (in seconds)
		virtual double getTotalTime() const { return m_T1 + m_T2 + m_T3; }

		/// @brief Return the times of the different phases of the ramp move
		virtual double T1() const { return m_T1; }
		virtual double T2() const { return m_T2; }
		virtual double T3() const { return m_T3; }

		/// @brief Calculate the necessary a and v of a rampmove, so that the move will take the desired time
		//static void calculateAV(double x0, double v0, double xtarget, double time, double T3, double amax, double& a, double& v);

	private:

		inline double sqr(double a) const { return a*a; }


		AbsPos m_x0;
		AbsPos m_xtarget;
		AbsPos m_richtung;
		double m_amax, m_vmax;

		double m_T1, m_T2, m_T3;
		// Folgende Werte beziehen sich nun auf den Wegparameter s:
		double m_a1, m_v2, m_a3;
};
		*/
#endif
