/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 

#ifndef _TimeStamp_H
#define _TimeStamp_H

#include <time.h>
#include <cob_utilities/StrUtil.h>

//-------------------------------------------------------------------

/** Measure system time.
 * Use this class for measure system time accurately. Under Windows, it uses
 * QueryPerformanceCounter(), which has a resolution of approx. one micro-second.
 * The difference between two time stamps can be calculated.
 */
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


#endif

