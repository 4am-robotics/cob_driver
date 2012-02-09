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
 * ROS package name: base_drive_chain
 * Description: custom Mutex implementation
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Feb 2009
 * ToDo: - Remove this class
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

