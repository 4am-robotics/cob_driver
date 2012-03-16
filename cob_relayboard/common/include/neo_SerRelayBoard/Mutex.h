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

