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
 

#ifndef WINDOWS_H
#define WINDOWS_H


#include <sys/select.h>

inline void Sleep(long dwMilliseconds)
{
	::timeval sleepTime = {0, dwMilliseconds * 1000};
	::select(0, 0, 0, 0, &sleepTime);
}


//#ifndef HANDLE
//typedef int HANDLE;
//#endif
//typedef int DWORD;
typedef unsigned char BYTE;
enum {
	FALSE = false,
	TRUE = true
};


inline int min(int a, int b)
{
	return (a < b) ? a : b;
}


inline int max(int a, int b)
{
	return (a > b) ? a : b;
}



#endif

