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
 * ROS package name: cob_canopen_motor
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Feb 2010
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


#ifndef DRIVEPARAM_INCLUDEDEF_H
#define DRIVEPARAM_INCLUDEDEF_H

//-----------------------------------------------


/**
 * Parameters and conversion functionality of a motor drive.
 * \ingroup DriversCanModul	
 */
class DriveParam
{

private:

	int m_iDriveIdent;
	int m_iEncIncrPerRevMot;	// encoder increments per revolution motor shaft 
	double m_dVelMeasFrqHz;		// only used for Neo drive, else = 1
	double m_dGearRatio;		//
	double m_dBeltRatio;		// if drive has belt set ratio, else = 1
	int m_iSign;				// direction of motion
	double m_dVelMaxEncIncrS;	// max. veloctiy
	double m_dAccIncrS2;		// max. acceleration
	double m_dDecIncrS2;		// max. decelration
	double m_dPosGearRadToPosMotIncr;
	int m_iEncOffsetIncr;		// Position in Increments of Steerwheel when Homingposition
					//  is reached (only needed forCoB3)
	int m_iHomingDigIn; // specifies which digital input is used for homing signal, standart 11 is good for COB3, 19 for Cob3_5
	bool m_bIsSteer;		// needed to distinguish motors for initializing
    double m_dCurrToTorque;		// factor to convert motor active current [A] into torque [Nm]
	double m_dCurrMax;		// max. current allowed

public:

	/**
	 * Default constructor.
	 */
	DriveParam()
	{
	
		m_bIsSteer = true; //has to be set, because it is checked for absolute / relative positioning
	
	}

	/**
	 * Sets the drive parameters.
	 * @param iDriveIdent identifier for the drive
	 * @param iEncIncrPerRevMot encoder increments per revolution of the motor shaft
	 * @param dVelMeasFrqHz set this value to 1
	 * @param dGearRatio ratio of the gear
	 * @param iSign change -1 for changing the motion direction
	 * @param dVelMaxEncIncrS maximum velocity given in encoder increments per second
	 * @param dAccIncrS2 acceleration in encoder increments per s^2
	 * @param dDecIncrS2 deceleration in encoder increments per s^2
	 */

	void setParam(
		int iDriveIdent,
		int iEncIncrPerRevMot,
		double dVelMeasFrqHz,
		double dBeltRatio,
		double dGearRatio,
		int iSign,
		double dVelMaxEncIncrS,
		double dAccIncrS2,
		double dDecIncrS2)
	{

		m_iDriveIdent = iDriveIdent;
		m_iEncIncrPerRevMot = iEncIncrPerRevMot;
		m_dVelMeasFrqHz = dVelMeasFrqHz;
		m_dBeltRatio = dBeltRatio;
		m_dGearRatio = dGearRatio;
		m_iSign = iSign;
		m_dVelMaxEncIncrS = dVelMaxEncIncrS;
		m_dAccIncrS2 = dAccIncrS2;
		m_dDecIncrS2 = dDecIncrS2;
		
		m_iHomingDigIn = 11; //for Cob3
		
		double dPI = 3.14159265358979323846;

		m_dPosGearRadToPosMotIncr = m_iEncIncrPerRevMot * m_dGearRatio
			* m_dBeltRatio / (2. * dPI);
	}

	//Overloaded Method for CoB3
	void setParam(
		int iDriveIdent,
		int iEncIncrPerRevMot,
		double dVelMeasFrqHz,
		double dBeltRatio,
		double dGearRatio,
		int iSign,
		double dVelMaxEncIncrS,
		double dAccIncrS2,
		double dDecIncrS2,
		int iEncOffsetIncr,
		bool bIsSteer,
        double dCurrToTorque,
		double dCurrMax)
	{

		m_iDriveIdent = iDriveIdent;
		m_iEncIncrPerRevMot = iEncIncrPerRevMot;
		m_dVelMeasFrqHz = dVelMeasFrqHz;
		m_dBeltRatio = dBeltRatio;
		m_dGearRatio = dGearRatio;
		m_iSign = iSign;
		m_dVelMaxEncIncrS = dVelMaxEncIncrS;
		m_dAccIncrS2 = dAccIncrS2;
		m_dDecIncrS2 = dDecIncrS2;
		m_iEncOffsetIncr = iEncOffsetIncr;
		m_bIsSteer = bIsSteer;
		
		m_iHomingDigIn = 11; //for Cob3

		double dPI = 3.14159265358979323846;

		m_dPosGearRadToPosMotIncr = m_iEncIncrPerRevMot * m_dGearRatio
			* m_dBeltRatio / (2. * dPI);

        m_dCurrToTorque = dCurrToTorque;
		m_dCurrMax = dCurrMax;
	}
	
	//Overloaded Method for CoB3 including new feature HomingDigIn, for compatibility reasons overloaded
	void setParam(
		int iDriveIdent,
		int iEncIncrPerRevMot,
		double dVelMeasFrqHz,
		double dBeltRatio,
		double dGearRatio,
		int iSign,
		double dVelMaxEncIncrS,
		double dAccIncrS2,
		double dDecIncrS2,
		int iEncOffsetIncr,
		bool bIsSteer,
        double dCurrToTorque,
		double dCurrMax,
		int iHomingDigIn)
	{

		m_iDriveIdent = iDriveIdent;
		m_iEncIncrPerRevMot = iEncIncrPerRevMot;
		m_dVelMeasFrqHz = dVelMeasFrqHz;
		m_dBeltRatio = dBeltRatio;
		m_dGearRatio = dGearRatio;
		m_iSign = iSign;
		m_dVelMaxEncIncrS = dVelMaxEncIncrS;
		m_dAccIncrS2 = dAccIncrS2;
		m_dDecIncrS2 = dDecIncrS2;
		m_iEncOffsetIncr = iEncOffsetIncr;
		m_bIsSteer = bIsSteer;

		double dPI = 3.14159265358979323846;

		m_dPosGearRadToPosMotIncr = m_iEncIncrPerRevMot * m_dGearRatio
			* m_dBeltRatio / (2. * dPI);

        m_dCurrToTorque = dCurrToTorque;
		m_dCurrMax = dCurrMax;
		m_iHomingDigIn = iHomingDigIn;
	}

	/**
	 * Returns the identifier of the drive.
	 */
	int getDriveIdent()
	{
		return m_iDriveIdent;
	}
	
	/**
	 * Returns the sign for the motion direction.
	 */
	int getSign()
	{
		return m_iSign;
	}
	
	/**
	 * Gets the maximum velocity of the drive in increments per second.
	 */
	double getVelMax()
	{
		return m_dVelMaxEncIncrS;
	}
	
	/**
	 * Converts position and velocity.
	 * @param dPosRad position in radiant
	 * @param dVelRadS velocity in radiant per seconds
	 * @param piPosIncr converted position in increments
	 * @param piVelIncrPeriod converted velocity in increments of period
	 */
	void PosVelRadToIncr(double dPosRad, double dVelRadS, int* piPosIncr, int* piVelIncrPeriod)
	{
		*piPosIncr = PosGearRadToPosMotIncr(dPosRad);
		*piVelIncrPeriod = VelGearRadSToVelMotIncrPeriod(dVelRadS);
	}

	/**
	 * Converts the temperature in degree Celsius.
	 * The temperature measure is only supported for the drive neo.
	 * @param iTempIncr temperature in a special internal unit
	 */
	int TempMeasIncrToGradCel(int iTempIncr)
	{
		double dTempMeasGradCel;

		dTempMeasGradCel = 0.0002 * (iTempIncr * iTempIncr) - 0.2592 * iTempIncr + 105;

		return (int)dTempMeasGradCel;
	}

	/**
	 * Converts revolution angle form radian to encoder increments.
	 * @param dPosGearRad angle in radian
	 */
	int PosGearRadToPosMotIncr(double dPosGearRad)
	{
		return ((int)(dPosGearRad * m_dPosGearRadToPosMotIncr));
	}
	
	/// Conversions of encoder increments to gear position in radians.
	double PosMotIncrToPosGearRad(int iPosIncr)
	{
		return ((double)iPosIncr / m_dPosGearRadToPosMotIncr);
	}
	
	/// Conversions of gear velocity in rad/s to encoder increments per measurment period.
	int VelGearRadSToVelMotIncrPeriod(double dVelGearRadS)
	{
		return ((int)(dVelGearRadS * m_dPosGearRadToPosMotIncr / m_dVelMeasFrqHz));
	}
	
	/// Conversions of  encoder increments per measurment period to gear velocity in rad/s.
	double VelMotIncrPeriodToVelGearRadS(int iVelMotIncrPeriod)
	{
		return ((double)iVelMotIncrPeriod / m_dPosGearRadToPosMotIncr * m_dVelMeasFrqHz);
	}
	
	/**
	 * Set the maximum acceleration.
	 * @param dMaxAcc Maximum acceleration
	 */
	void setMaxAcc(double dMaxAcc)
	{
		m_dAccIncrS2 = dMaxAcc;
	}

	/**
	 * Get the maximum acceleration.
	 * @return Maximum acceleration
	 */
	double getMaxAcc()
	{
		return m_dAccIncrS2;
	}

	/**
	 * Set the maximum deceleration.
	 * @param dMaxAcc Maximum deceleration
	 */
	void setMaxDec(double dMaxDec)
	{
		m_dDecIncrS2 = dMaxDec;
	}

	/**
	 * Get the maximum deceleration.
	 * @return Maximum deceleration
	 */
	double getMaxDec()
	{
		return m_dDecIncrS2;
	}

	/**
	 * Set the maximum velocity.
	 * @param dMaxVel Maximum velocity
	 */
	void setMaxVel(double dMaxVel)
	{
		m_dVelMaxEncIncrS = dMaxVel;
	}

	/**
	 * Get the maximum velocity in increments per second.
	 * @return Maximum velocity [inc/sec].
	 */
	double getMaxVel()
	{
		return m_dVelMaxEncIncrS;
	}
	
	/**
	 * Get the gear ratio.
	 * @return The gear ratio.
	 */
	double getGearRatio()
	{
		return m_dGearRatio;
	}
	/**
	 * Get the belt ratio.
	 * @return The belt ratio.
	 */
	double getBeltRatio()
	{
		return m_dBeltRatio;
	}
	
	/**
	 * Get the EncoderOffset
	 * @return the Encoderoffset
	 */
	int getEncOffset()
	{
		return m_iEncOffsetIncr;
	}
	
	/**
	 * Get the DriveType - If it's a Steering or Driving Motor
	 * @return the Encoderoffset
	 */
	bool getIsSteer()
	{
		return m_bIsSteer;
	}
	/**
	 * Get the DriveType - If it's a Steering or Driving Motor
	 * @return the Encoderoffset
	 */
	int getEncIncrPerRevMot()
	{
		return m_iEncIncrPerRevMot;
	}
	/**
	 * Get factor to convert motor active current [A] into torque [Nm]	 
	 */
	double getCurrToTorque()
	{
		return m_dCurrToTorque;
	}
	/**
	 * Get maximum current allowed
	 */
	double getCurrMax()
	{
		return m_dCurrMax;
	}
	/**
	 * Get digital Input for Homing signal
	 */
	int getHomingDigIn()
	{
		return m_iHomingDigIn;
	}
	/**
	 * Set digital Input for Homing signal
	 */
	void setHomingDigIn(int HomingDigIn)
	{
		m_iHomingDigIn = HomingDigIn;
	}
};
//-----------------------------------------------
#endif
