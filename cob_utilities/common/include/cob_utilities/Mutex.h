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
 

#ifndef MUTEX_INCLUDEDEF_H
#define MUTEX_INCLUDEDEF_H
//-----------------------------------------------
#include <pthread.h>

const unsigned int INFINITE = 0;


class Mutex
{
private:
	pthread_mutex_t m_hMutex;

public:
	Mutex()
	{
		pthread_mutex_init(&m_hMutex, 0);
	}

	Mutex( std::string sName)
	{
// no named Mutexes for POSIX
		pthread_mutex_init(&m_hMutex, 0);
	}

	~Mutex()
	{
		pthread_mutex_destroy(&m_hMutex);
	}

	/** Returns true if log was successful.
	 */
	bool lock( unsigned int uiTimeOut = INFINITE )
	{
		int ret;

		if (uiTimeOut == INFINITE)
		{
			ret = pthread_mutex_lock(&m_hMutex);
		}
		else
		{
			timespec abstime = { time(0) + uiTimeOut, 0 };
			ret = pthread_mutex_timedlock(&m_hMutex, &abstime);
		}

		return ! ret;
	}

	void unlock()
	{
		pthread_mutex_unlock(&m_hMutex);
	}
};
//-----------------------------------------------
#endif

