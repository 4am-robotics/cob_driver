//-----------------------------------------------
// Neobotix
// www.neobotix.de
// Copyright (c) 2003. All rights reserved.

// author:
//-----------------------------------------------

#ifndef _TimeStamp_H
#define _TimeStamp_H

#include <time.h>
#include "StrUtil.h"

//-------------------------------------------------------------------

/** Measure system time with very high accuracy.
 * Use this class for measure system time accurately. Under Windows, it uses
 * QueryPerformanceCounter(), which has a resolution of approx. one micro-second.
 * The difference between two time stamps can be calculated.
 */
 
namespace Neobotix
{
class TimeStamp
{
	public:
		/// Constructor.
		TimeStamp();

		/// Destructor.
		virtual ~TimeStamp() {};

		/// Makes time measurement.
		void SetNow();

		/// Retrieves time difference in seconds.
		double operator- ( const TimeStamp& EarlierTime ) const;

		/// Increase the timestamp by TimeS seconds.
		/** @param TimeS must be >0!.
		 */
		void operator+= ( double TimeS );

		/// Reduces the timestamp by TimeS seconds.
		/** @param TimeS must be >0!.
		 */
		void operator-= ( double TimeS );

		/// Checks if this time is after time "Time".
		bool operator> ( const TimeStamp& Time );

		/// Checks if this time is before time "Time".
		bool operator< ( const TimeStamp& Time );

		/**
		 * Gets seconds and nanoseconds of the timestamp.
		 */
		void getTimeStamp ( long& lSeconds, long& lNanoSeconds );

		/**
		 * Sets timestamp from seconds and nanoseconds.
		 */
		void setTimeStamp ( const long& lSeconds, const long& lNanoSeconds );

		/**
		 * return the current time as string, in long format YYYY-MM-DD HH:MM:SS.ssssss
		 *** Attention *** call SetNow() before calling this function
		 */
		std::string CurrentToString();

		std::string ToString();

	protected:

		/// Internal time stamp data.
		timespec m_TimeStamp;

	private:

		/// Conversion timespec -> double
		static double TimespecToDouble ( const ::timespec& LargeInt );

		/// Conversion double -> timespec
		static ::timespec DoubleToTimespec ( double TimeS );

};
};//namespace Neobotix
#endif

