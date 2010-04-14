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
 * Author: Matthias Bengel, email:matthias.bengel@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Feb 2009
 * ToDo: Check if this can be removed
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

#include <cob_canopen_motor/StrUtil.h>
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

