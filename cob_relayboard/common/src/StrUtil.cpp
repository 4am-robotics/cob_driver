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


#include <neo_SerRelayBoard/StrUtil.h>
#include <algorithm>

std::string StringToUpper(std::string strToConvert) {
	for(unsigned int i=0;i<strToConvert.length();i++)
	{
		strToConvert[i] = toupper(strToConvert[i]);
	}
	return strToConvert;
}

std::string StringToLower(std::string strToConvert) {
	for(unsigned int i=0;i<strToConvert.length();i++)
	{
		strToConvert[i] = tolower(strToConvert[i]);
	}
	return strToConvert;
}

std::string NumToString(const int n) {
	std::stringstream ss;
	ss << n;
	return ss.str();
}

std::string NumToString(const unsigned int n) {
	std::stringstream ss;
	ss << n;
	return ss.str();
}

std::string NumToString(const long l) {
	std::stringstream ss;
	ss << l;
	return ss.str();
}

std::string NumToString(const float f, unsigned int width, unsigned int precise) {
	std::stringstream ss;
	ss << std::setw(width) << std::setprecision(precise) << f;
	return ss.str();
}

std::string NumToString(const double d, unsigned int width, unsigned int precise) {
	std::stringstream ss;
	ss << std::setw(width) << std::setprecision(precise) << d;
	return ss.str();
}

/**
 * C++ version char* style "itoa":
 */
char* itoa( int value, char* result, int base ) 
{
        // check that the base if valid
	if (base < 2 || base > 16) { 
		*result = 0; return result; 
	}

	char* out = result;
	int quotient = value;
	
	do {
		*out = "0123456789abcdef"[ std::abs( quotient % base ) ];
		++out;
		quotient /= base;
	} while ( quotient );

        // Only apply negative sign for base 10
	if ( value < 0 && base == 10) *out++ = '-';

	std::reverse( result, out );
	*out = 0;
	return result;
}

/**
 * C++ version std::string style "itoa":
 */
std::string itoa(int value, int base) 
{
	enum { kMaxDigits = 35 };
	std::string buf;
	
	buf.reserve( kMaxDigits ); // Pre-allocate enough space.
	
	// check that the base if valid
	if (base < 2 || base > 16) return buf;
	
	int quotient = value;

	// Translating number to string with base:
	do {
		buf += "0123456789abcdef"[ std::abs( quotient % base ) ];
		quotient /= base;
	} while ( quotient );
	
	// Append the negative sign for base 10
	if ( value < 0 && base == 10) buf += '-';
	std::reverse( buf.begin(), buf.end() );
	return buf;
}

