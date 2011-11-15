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
 * ROS stack name: cob3_common
 * ROS package name: cob3_utilities
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

#ifndef MATHSUP_INCLUDEDEFX_H
#define MATHSUP_INCLUDEDEFX_H

#include <math.h>


//-----------------------------------------------
/**
 * Provides diverse mathematical utilities and functions.
 * \ingroup MathUtilsModul
 */
class MathSup
{
public:
	// -------- Constants
	/// Constant for PI.
	static const double PI;

	/// Constant for 2*PI
	static const double TWO_PI;

	/// Constant for PI/2
	static const double HALF_PI;

	// -------- Angle conversion
	/**
	 * Converts radian to degree.
	 */
	static double convRadToDeg(const double& dAngRad)
	{
		return (dAngRad * 180.0 / PI);
	}

	/**
	 * Converts degree to radian.
	 */
	static double convDegToRad(const double& dAngDeg)
	{
		return ( dAngDeg * PI / 180.0 );
	}

	/**
	 * Normalizes angle to the interval [0,2pi[.
	 */
	static void normalize2Pi(double& angle)
	{
		angle-=floor(angle/(2 * PI))* 2 * PI;
	}

	/** 
	 * Normalizes angle to the interval ]-pi,pi].
	 */
	static void normalizePi(double& angle)
	{
		normalize2Pi(angle);
		if ( angle> PI ) angle-=2 * PI;
	}

	/**
	 * Normalizes angle to the interval ]-pi/2,pi/2] for lines or segments.
	 */
	static void normalizePiHalf(double& angle)
	{
		normalize2Pi(angle);
		if (angle > PI) angle-=2*PI;
		if (angle > PI/2.0) angle -= PI;
		if (angle <= -PI/2.0) angle += PI;
	}

	/**
	 * Returns the sign.
	 */
	static double sign(const double& x)
	{
		if (x<0)
			return -1.0;
		else
			return 1.0;
	}

	/**
	 * Returns the minimum.
	 */
	static double getMin(const double& a, const double& b)
	{
		if (a < b)
			return a;
		else
			return b;
	}

	/**
	 * Returns the maximum.
	 */
	static double getMax(const double& a, const double& b)
	{
		if (a > b)
			return a;
		else
			return b;
	}

	/**
	 * Calculates the difference angle a-b.
	 * The difference ange is normalized to the interval ]-pi,pi].
	 */
	static double calcDeltaAng(const double& a, const double& b)
	{
		double c = a-b;
		normalizePi(c);
		return c;
	}


	/**
	 * Calculates the arcus tangens and removes ambiguity in quadrant.
	 */
	static double atan4quad(double y, double x)
	{
		double result;

		if((x==0.0) && (y==0.0))
			result = 0.0;
		else if((x==0.0) && (y > 0.0))
			result = HALF_PI;
		else if((x==0.0) && (y < 0.0))
			result = -HALF_PI;
		else if((y==0.0) && (x > 0.0))
			result = 0.0;
		else if((y==0.0) && (x < 0.0))
			result = PI;
		else
		{
			result = atan(y/x);
			if(x<0.0)
				{
					if(y>0.0)		// Quadrant 2 -> correct
						result += PI;
					else			// Quadrant 3 -> correct
						result -= PI;
				}		
		}	
		normalizePi(result);
		return result;
	}


	/**
	 * Calculates the euclidean distance of two points.
	 */
	static double distance(double x1, double y1, double x2, double y2)
	{
		return sqrt( distanceSq( x1, y1, x2, y2 ) );
	}

	/**
	 * Calculates the squared euclidean distance of two points.
	 */
	static double distanceSq(double x1, double y1, double x2, double y2)
	{
		double dx = x2 - x1;
		double dy = y2 - y1;

		return dx*dx + dy*dy;
	}

	/**
	 * Checks if a bit is set.
	 */
	static bool isBitSet(int iVal, int iNrBit)
	{
		if( (iVal & (1 << iNrBit)) == 0)
			return false;
		else
			return true;
	}
	
	/**
	 * Converts a float to a 4 byte integer value according to
	 * IEEE specification.
	 */
	static double convFloatToInt4Byte(double dVal)
	{
		//Todo 
		return 1.0;
	}

	/**
	 * Converts a 4 byte integer value to float according to
	 * IEEE specification.
	 */
	static double convInt4ByteToFloat(int iVal)
	{
		unsigned char c[4];
		double dSign;
		double dFraction;
		double dExp;
		double dVal;

		c[0] = iVal >> 24;
		c[1] = iVal >> 16;
		c[2] = iVal >> 8;
		c[3] = iVal;

		if( (c[0] & 0x80) == 0)
			dSign = 1;
		else
			dSign = -1;

		dFraction = (iVal & 0xFFFFFF) | 0x800000;
		dFraction = dFraction * pow(2., -23);

		dExp = ((c[0] << 1) | (c[1] >> 7)) - 127;

		dVal = dSign * dFraction * pow(10., dExp);
		
		return dVal;
	}

	/**
	 * Limits a variable to the interval [-dLimit, dLimit].
	 * @param pdToLimit variable to be limited
	 * @param dLimit bound of the interval [-dLimit, dLimit]
	 * @return 0: value is in the interval, 1: value has been bound to the lower bound,
	 * 2: value has been bound to the upper bound
	 */
	static int limit(double* pdToLimit, double dLimit)
	{
		int iRet = 0;

		if(*pdToLimit < -dLimit)
		{
			*pdToLimit = -dLimit;
			iRet = 1;
		}
		if(*pdToLimit > dLimit)
		{
			*pdToLimit = dLimit;
			iRet = 2;
		}

		return iRet;
	}

	/**
	 * Limits a variable to the interval [-dLimit, dLimit].
	 * @param piToLimit variable to be limited
	 * @param iLimit bound of the interval [-dLimit, dLimit]
	 * @return 0: value is in interval, 1: value has been bound to the lower bound,
	 * 2: value has been bound to the upper bound
	 */
	static int limit(int* piToLimit, int iLimit)
	{
		int iRet = 0;

		if(*piToLimit < -iLimit)
		{
			*piToLimit = -iLimit;
			iRet = 1;
		}
		if(*piToLimit > iLimit)
		{
			*piToLimit = iLimit;
			iRet = 2;
		}

		return iRet;
	}

	/**
	 * Checks value to be in an interval [dLow, dHigh].
	 * @param dLow lower bound
	 * @param dHigh upper bound
	 * @param dVal value
	 * @return true if value is in the interval
	 */
	static bool isInInterval(double dLow, double dHigh, double dVal)
	{
		if( (dVal >= dLow) && (dVal <= dHigh) )
			return true;

		else
			return false;
	}

};


//-----------------------------------------------
#endif
