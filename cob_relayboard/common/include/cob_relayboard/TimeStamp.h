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
 * ROS package name: generic_can
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Feb 2009
 * ToDo: Check if this is still neccessary. Can we use the ROS-Infrastructure within the implementation?
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

#ifndef _TimeStamp_H
#define _TimeStamp_H

#include <time.h>
#include <cob_relayboard/StrUtil.h>

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

