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
 * Description:
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

#ifdef __LINUX__
#include <cob_powercube_chain/TimeStamp.h>
#else
#include <windows.h>
#include <cob_powercube_chain/TimeStamp.h>

namespace RTB {
#endif

//-----------------------------------------------------------------------------
#ifdef __LINUX__
TimeStamp::TimeStamp()
{
	m_TimeStamp.tv_sec = 0;
	m_TimeStamp.tv_nsec = 0;
}

void TimeStamp::SetNow()
{
	::clock_gettime(CLOCK_REALTIME, &m_TimeStamp);
}

double TimeStamp::TimespecToDouble(const ::timespec& LargeInt)
{
	return double(LargeInt.tv_sec) + double(LargeInt.tv_nsec) / 1e9;
}

::timespec TimeStamp::DoubleToTimespec(double TimeS)	
{	
	::timespec DeltaTime;
	if (! ( TimeS < 4e9 && TimeS > 0.0 ))
	{
		DeltaTime.tv_sec = 0;
		DeltaTime.tv_nsec = 0;
		return DeltaTime;
    	}

	DeltaTime.tv_sec = ::time_t(TimeS);
	DeltaTime.tv_nsec
		= static_cast<long int>((TimeS - double(DeltaTime.tv_sec)) * 1e9);

	return DeltaTime;
}

double TimeStamp::operator-(const TimeStamp& EarlierTime) const
{
	::timespec Res;

	Res.tv_sec = m_TimeStamp.tv_sec - EarlierTime.m_TimeStamp.tv_sec;
	Res.tv_nsec = m_TimeStamp.tv_nsec - EarlierTime.m_TimeStamp.tv_nsec;

	if (Res.tv_nsec < 0) {
		Res.tv_sec--;
		Res.tv_nsec += 1000000000;
	}

	return TimespecToDouble(Res);
}

void TimeStamp::operator+=(double TimeS)
{
	::timespec Dbl = DoubleToTimespec(TimeS);
	m_TimeStamp.tv_sec += Dbl.tv_sec;
	m_TimeStamp.tv_nsec += Dbl.tv_nsec;
	if (m_TimeStamp.tv_nsec > 1000000000)
	{
		m_TimeStamp.tv_sec++;
		m_TimeStamp.tv_nsec -= 1000000000;
	}
}

void TimeStamp::operator-=(double TimeS)
{
	::timespec Dbl = DoubleToTimespec(TimeS);
	m_TimeStamp.tv_sec -= Dbl.tv_sec;
	m_TimeStamp.tv_nsec -= Dbl.tv_nsec;
	if (m_TimeStamp.tv_nsec < 0.0)
	{
		m_TimeStamp.tv_sec--;
		m_TimeStamp.tv_nsec += 1000000000;
	}
}

bool TimeStamp::operator>(const TimeStamp& Time)
{
	if (m_TimeStamp.tv_sec > Time.m_TimeStamp.tv_sec) return true;
	if ((m_TimeStamp.tv_sec == Time.m_TimeStamp.tv_sec) &&
		(m_TimeStamp.tv_nsec > Time.m_TimeStamp.tv_nsec)) return true;
	return false;
}

bool TimeStamp::operator<(const TimeStamp& Time)
{
	if (m_TimeStamp.tv_sec < Time.m_TimeStamp.tv_sec) return true;
	if ((m_TimeStamp.tv_sec == Time.m_TimeStamp.tv_sec) &&
		(m_TimeStamp.tv_nsec < Time.m_TimeStamp.tv_nsec)) return true;
	return false;
}

void TimeStamp::getTimeStamp(long& lSeconds, long& lNanoSeconds)
{
	lSeconds = m_TimeStamp.tv_sec;
	lNanoSeconds = m_TimeStamp.tv_nsec;
};

void TimeStamp::setTimeStamp(const long& lSeconds, const long& lNanoSeconds)
{
	m_TimeStamp.tv_sec = lSeconds;
	m_TimeStamp.tv_nsec = lNanoSeconds;
};

#else
//-------------------------------------------------------------------

LARGE_INTEGER ClockFrequency;
BOOL CounterHW = QueryPerformanceFrequency( &ClockFrequency );
double TimeStamp::m_SecPerCount = 1.0 / TimeStamp::LargeIntToDouble(ClockFrequency);
double TimeStamp::m_DwordSize = double(0xFFFFFFFF);

//-----------------------------------------------------------------------------

TimeStamp::TimeStamp()
{
	m_TimeStamp.HighPart = 0;
	m_TimeStamp.LowPart = 0;
}

void TimeStamp::SetNow()
{
	QueryPerformanceCounter( &m_TimeStamp );
}

double TimeStamp::LargeIntToDouble(const LARGE_INTEGER& LargeInt)
{
	return LargeInt.HighPart * m_DwordSize + LargeInt.LowPart;
}

LARGE_INTEGER TimeStamp::DoubleToLargeInt(double TimeS)	
{	
	LARGE_INTEGER DeltaTime;
	DeltaTime.LowPart = 0;
	DeltaTime.HighPart = 0;
	if (! ( TimeS < 4e9 && TimeS > 0.0 )) return DeltaTime;
	DeltaTime.LowPart = (unsigned long)(TimeS / m_SecPerCount);
	return DeltaTime;
}

double TimeStamp::operator-(const TimeStamp& EarlierTime) const
{
	LARGE_INTEGER Res;
	Res.HighPart = m_TimeStamp.HighPart - EarlierTime.m_TimeStamp.HighPart;
	Res.LowPart = m_TimeStamp.LowPart - EarlierTime.m_TimeStamp.LowPart;
	if (EarlierTime.m_TimeStamp.LowPart > m_TimeStamp.LowPart)
		Res.HighPart--;
	return LargeIntToDouble(Res) * m_SecPerCount;
}

void TimeStamp::operator+=(double TimeS)
{
	LARGE_INTEGER& Dbl = DoubleToLargeInt(TimeS);
	LARGE_INTEGER Res;
	Res.LowPart = m_TimeStamp.LowPart + Dbl.LowPart;
	if ((Res.LowPart < m_TimeStamp.LowPart) || (Res.LowPart < Dbl.LowPart))
		m_TimeStamp.HighPart++;
	m_TimeStamp.LowPart = Res.LowPart;
}

void TimeStamp::operator-=(double TimeS)
{
	LARGE_INTEGER& Dbl = DoubleToLargeInt(TimeS);
	LARGE_INTEGER Res;
	Res.LowPart = m_TimeStamp.LowPart - Dbl.LowPart;
	Res.HighPart = m_TimeStamp.HighPart - Dbl.HighPart;
	if ((Res.LowPart > m_TimeStamp.LowPart))
		Res.HighPart--;
	m_TimeStamp.LowPart = Res.LowPart;
	m_TimeStamp.HighPart = Res.HighPart;
}

bool TimeStamp::operator>(const TimeStamp& Time)
{
	if (m_TimeStamp.HighPart > Time.m_TimeStamp.HighPart) return true;
	if ((m_TimeStamp.HighPart == Time.m_TimeStamp.HighPart) &&
		(m_TimeStamp.LowPart > Time.m_TimeStamp.LowPart)) return true;
	return false;
}

bool TimeStamp::operator<(const TimeStamp& Time)
{
	if (m_TimeStamp.HighPart < Time.m_TimeStamp.HighPart) return true;
	if ((m_TimeStamp.HighPart == Time.m_TimeStamp.HighPart) &&
		(m_TimeStamp.LowPart < Time.m_TimeStamp.LowPart)) return true;
	return false;
}
}
#endif

