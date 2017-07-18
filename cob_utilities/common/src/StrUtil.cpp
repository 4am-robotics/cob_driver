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
 

#include <cob_utilities/StrUtil.h>
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

