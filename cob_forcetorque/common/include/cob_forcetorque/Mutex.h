//-----------------------------------------------
// Neobotix 
// www.neobotix.de
// Copyright (c) 2003. All rights reserved.

// author: Oliver Barth
//-----------------------------------------------

#ifndef MUTEX_INCLUDEDEF_H
#define MUTEX_INCLUDEDEF_H
//-----------------------------------------------
#include <pthread.h>
#include <string>

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

