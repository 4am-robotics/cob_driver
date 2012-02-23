/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef DRIVEPARAM_INCLUDEDEF_H
#define DRIVEPARAM_INCLUDEDEF_H

//-----------------------------------------------

const double PI = 3.14159265358979323846;

/**
 * Parameters and conversion functionality of a motor drive.
 */
class DriveParam
{
public:

	enum TypeEncoder
	{
		ENCODER_NO,
		ENCODER_INCREMENTAL,
		ENCODER_ABSOLUTE
	};

	/**
	 *
	 */
	DriveParam()
	{
		m_iDriveIdent = 0;
		m_iEncIncrPerRevMot = 0;
		m_dVelMeasFrqHz = 0;
		m_dGearRatio = 0;
		m_dBeltRatio = 0;
		m_iSign = 0;
		m_bHoming = 0;
		m_dHomePos = 0;
		m_dHomeVel = 0;
		m_iHomeEvent = 0;
		m_iHomeDigIn = 0;
		m_iHomeTimeOut = 0;
		m_dVelMaxEncIncrS = 0;
		m_dVelPModeEncIncrS = 0;
		m_dAccIncrS2 = 0;
		m_dDecIncrS2 = 0;
		m_iCANId = 0;
		m_iTypeEncoder = ENCODER_INCREMENTAL;
		m_bUsePosMode = false;
		m_bEnabled = false;

		m_dRadToIncr = 0;
	}

	/**
	 * Sets the drive parameters.
	 */
	void set(		int iDriveIdent,
				int iEncIncrPerRevMot,
				double dVelMeasFrqHz,
				double dBeltRatio, double dWheelRatio,
				int iSign,
				bool bHoming, double dHomePos, double dHomeVel, int iHomeEvent, int iHomeDigIn, int iHomeTimeOut,
				double dVelMaxEncIncrS, double dVelPModeEncIncrS,
				double dAccIncrS2, double dDecIncrS2,
				int iTypeEncoder,
				int iCANId,
				bool bUsePosMode,
				bool bEnabled )
	{
		m_iDriveIdent = iDriveIdent;
		m_iEncIncrPerRevMot = iEncIncrPerRevMot;
		m_dVelMeasFrqHz = dVelMeasFrqHz;
		m_dBeltRatio = dBeltRatio;
		m_dGearRatio = dWheelRatio;
		m_iSign = iSign;
		m_bHoming = bHoming;
		m_dHomePos = dHomePos;
		m_dHomeVel = dHomeVel;
		m_iHomeEvent = iHomeEvent;
		m_iHomeDigIn = iHomeDigIn;
		m_iHomeTimeOut = iHomeTimeOut;
		m_dVelMaxEncIncrS = dVelMaxEncIncrS;
		m_dVelPModeEncIncrS = dVelPModeEncIncrS;
		m_dAccIncrS2 = dAccIncrS2;
		m_dDecIncrS2 = dDecIncrS2;
		m_iTypeEncoder = iTypeEncoder;
		m_iCANId = iCANId;
		m_bUsePosMode = bUsePosMode;
		m_bEnabled = bEnabled;
		
		m_dRadToIncr = (m_iEncIncrPerRevMot * m_dGearRatio * m_dBeltRatio) / (2. * PI);

		
		
	}

	/**
	 * Returns the identifier of the drive.
	 */
	int getDriveIdent() { return m_iDriveIdent; }

	/**
	 * Returns the CAN identifier of the drive.
	 */
	int getCANId() { return m_iCANId; }
	
	/**
	 * Returns the sign for the motion direction.
	 */
	int getSign() {	return m_iSign; }

	/**
	 *
	 */
	bool isEnabled() { return m_bEnabled; }

	/**
	 *
	 */
	bool usePosMode() { return m_bUsePosMode; }

	/**
	 *
	 */
	bool getHoming() { return m_bHoming; }

	/**
	 *
	 */
	void setHoming(bool b) { m_bHoming = b; }
	
	/**
	 *
	 */
	double getHomePos() { return m_dHomePos; }
	
	/**
	 *
	 */
	double getHomeVel() { return m_dHomeVel; }
	
	/**
	 *
	 */
	int getHomeEvent() { return m_iHomeEvent; }
	
	/**
	 *
	 */
	int getHomeDigIn() { return m_iHomeDigIn; }
	
	/**
	 *
	 */
	int getHomeTimeOut() { return m_iHomeTimeOut; }

	/**
	 * returns the maximum velocity of the drive in increments per second.
	 */
	double getVelMax() { return m_dVelMaxEncIncrS; }

	/**
	 * returns the maximum velocity of the drive in increments per second.
	 */
	double getVelPosMode() { return m_dVelPModeEncIncrS; }

	/**
	 *
	 */
	double getAcc()	{ return m_dAccIncrS2; }

	/**
	 *
	 */
	double getDec()	{ return m_dDecIncrS2; }

	/**
	 *
	 */
	int getTypeEncoder() { return m_iTypeEncoder; }

	/**
	 *
	 */
	int getEncoderIncr() { return m_iEncIncrPerRevMot; }

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
	 * Converts position and velocity.
	 * @param dPosRad			position in radians
	 * @param dVelRadS			velocity in radians per seconds
	 * @param piPosIncr			converted position in increments
	 * @param piVelIncrPeriod	converted velocity in increments per period
	 */
	void convRadSToIncrPerPeriod(double dPosRad, double dVelRadS, int* piPosIncr, int* piVelIncrPeriod)
	{
		*piPosIncr = (int)convRadToIncr(dPosRad);
		*piVelIncrPeriod = (int)convRadSToIncrPerPeriod(dVelRadS);
	}

	/// Conversions of wheel angle in radians to encoder increments.
	double convRadToIncr(double dPosWheelRad)
	{
		return (dPosWheelRad * m_dRadToIncr);
	}
	
	/// Conversions of encoder increments to wheel angle in radians.
	double convIncrToRad(int iPosIncr)
	{
		return ((double)iPosIncr / m_dRadToIncr);
	}
	
	/// Conversions of gear velocity in rad/s to encoder increments per measurement period.
	double convRadSToIncrPerPeriod(double dVelWheelRadS)
	{
		return ( (dVelWheelRadS * m_dRadToIncr) / m_dVelMeasFrqHz );
	}
	
	/// Conversions of encoder increments per measurment period to gear velocity in rad/s.
	double convIncrPerPeriodToRadS(int iVelMotIncrPeriod)
	{
		return ( (double)iVelMotIncrPeriod / m_dRadToIncr * m_dVelMeasFrqHz );
	}

private:

	int m_iDriveIdent;			// drive identifier 0 ... n
	int m_iEncIncrPerRevMot;	// encoder increments per revolution of motor shaft 
	double m_dVelMeasFrqHz;		// only used for Neo drive = 500, else = 1
	double m_dGearRatio;		// gear ratio
	double m_dBeltRatio;		// if drive has belt set ratio, else = 1
	int m_iSign;				// direction of motion
	bool m_bHoming;				// if true, motor starts homing sequence on startup
	double m_dHomePos;			// position set if homing switch active
	double m_dHomeVel;			// velocity while homing
	int m_iHomeEvent;			// type of homing input used at amplifier
	int m_iHomeDigIn;			// number of digital input used for homing
	int m_iHomeTimeOut;			// time out for homing
	double m_dVelMaxEncIncrS;	// max. veloctiy [encoder increments / s]
	double m_dVelPModeEncIncrS;	// velocity in position mode e. g. if amplifier generates trajectory
	double m_dAccIncrS2;		// max. acceleration [encoder increments / s�]
	double m_dDecIncrS2;		// max. deceleration [encoder increments / s�]
	int m_iTypeEncoder;			// see enum TypeEncoder
	int m_iCANId;				// CAN identifier
	bool m_bUsePosMode;			// if enabled use amplifier position mode if velocity command = 0
	bool m_bEnabled;			// if enabled, drive receives command data

	double m_dRadToIncr;

public:



};
//-----------------------------------------------
#endif
