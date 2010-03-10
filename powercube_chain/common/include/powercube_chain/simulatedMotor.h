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
 * Description: This class simulates a PowerCube.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Felix Geibel, email:Felix.Geibel@gmx.de
 * Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 *
 * Date of creation: Aug 2007
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

#ifndef _SIMULATED_MOTOR_H_
#define _SIMULATED_MOTOR_H_

#include <powercube_chain/moveCommand.h>
#include <string>

class simulatedMotor
{
	public:
		
		simulatedMotor(double lowLimit, double upLimit, double maxAcc, double maxVel);
		~simulatedMotor() {;}
		
		/// @brief initializes the module, if an error occurs function returns false
		virtual bool init() { return true; }

		/// @brief if error occured during init, get the error message with this
		virtual std::string getErrorMessage() { return std::string("No Errors."); }
		
		/// @brief any messages useful for debug are sent to this stream:
		// virtual void setDebugOutput(ostream* os) { deb = os; }
		
		/////////////////////////////////////////
		// Zunächst die Steuerungs-Funktionen: //
		/////////////////////////////////////////

		/// @brief executes a rampmove
		virtual void moveRamp(double targetAngle, double vmax, double amax);
	
		/// @brief Moves the motor with the given velocity
		virtual void moveVel(double vel);
		
		/// @brief Moves the motor with the given velocity
		virtual void movePos(double pos);
		
		/// @brief Stops the motor immediately
		virtual void stop();
		
		////////////////////////////////////////////////////////
		// Funktionen zum setzen und auslesen von Parametern: //
		////////////////////////////////////////////////////////
		
		/// @brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
		/// A Value of 0.5 is already pretty fast, you probably don't want anything more than 1.0...
		virtual void setMaxVelocity(double radpersec) { m_vmax = radpersec; }
		virtual double getMaxVelocity() { return m_vmax; }
		
		/// @brief Sets the maximum angular acceleration (rad/s^2) for the Joints, use with care!
		/// A Value of 0.5 is already pretty fast, you probably don't want anything more than 1.0...
		virtual void setMaxAcceleration(double radPerSecSquared) { m_amax = radPerSecSquared; }
		virtual double getMaxAcceleration() { return m_amax; }
		
		/// @brief sets the Joint Limits, the motor has to stay in these limits!
		virtual void setLimits(double lowerLimit, double upperLimit)
			{ m_ul = upperLimit; m_ll = lowerLimit; }
		virtual double getUpperLimit() { return m_ul; }
		virtual double getLowerLimit() { return m_ll; }
		
		/// @brief sets / gets the time that the motor needs to reach target velocity (moveVel)
		virtual void setTimeConstant(double T) { T0 = T; }
		virtual double getTimeConstant() const { return T0; }
		
		////////////////////////////////////////////
		// hier die Funktionen zur Statusabfrage: //
		////////////////////////////////////////////
		
		/// @brief Get a representation of the rampMove that the motor WOULD execute if told so at the time of function call
		virtual RampCommand getRampMove(double targetAngle, double v, double a);
		
		/// @brief Same but using the maximum velocity and acceleration of this motor
		virtual RampCommand getRampMove(double targetAngle) { return getRampMove(targetAngle, m_vmax, m_amax); }
		
		/// @brief Returns the current Joint Angles
		virtual double getAngle() { return m_lastMove.getPos(); }
		
		/// @brief Returns the current Angular velocities (Rad/s)
		virtual double getVelocity() { return m_lastMove.getVel(); }
		
		/// @brief Returns true if the Joint is still moving
		/// also returns true if Joints are accelerating or decelerating
		virtual bool statusMoving() { return m_lastMove.isActive(); }
		
		/// @brief Returns true if the Joint is decelerating at end of movement
		virtual bool statusDec() { return m_lastMove.inPhase3(); }
		
		/// @brief Returs true if the Joint is in phase one of rampmove
		virtual bool statusAcc() { return m_lastMove.inPhase1(); }
		
	private:
		
		RampCommand m_lastMove;
		
		double m_ul, m_ll;			// upper limit and lower limit
		double m_amax, m_vmax;		// Never mover faster than this!
		
		double T0;					// Zeitkonstante für Annäherung der Sprungantwort durch Rampe
		
		//ostream * deb;
};


#endif
