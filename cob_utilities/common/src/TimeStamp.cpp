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
 

#include <cob_utilities/TimeStamp.h>

//-----------------------------------------------------------------------------

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

std::string TimeStamp::CurrentToString()
{
# define TIME_SIZE 400

	const struct tm *tm;
	size_t len;
	time_t now;
	char pres[TIME_SIZE];
	std::string s;

	now = time ( NULL );
	tm = localtime ( &now );
	len = strftime ( pres, TIME_SIZE, "%Y-%m-%d %H:%M:%S.", tm );

	s = (std::string)pres + NumToString(m_TimeStamp.tv_nsec / 1000);

	return s;
# undef TIME_SIZE
}

std::string TimeStamp::ToString()
{
# define TIME_SIZE 4000

	const struct tm *tm;
	size_t len;
	//time_t now;
	char pres[TIME_SIZE];
	std::string s;

	tm = localtime ( &m_TimeStamp.tv_sec );
	len = strftime ( pres, TIME_SIZE, "%Y-%m-%d %H:%M:%S.", tm );

	s = (std::string)pres + NumToString(m_TimeStamp.tv_nsec / 1000);

	return s;
# undef TIME_SIZE
}
