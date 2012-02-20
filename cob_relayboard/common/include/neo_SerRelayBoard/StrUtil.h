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


#ifndef STRUTIL_H
#define STRUTIL_H

#include <sstream>
#include <iomanip>

/** change the string to upper case
    @param strToConvert the string to be converted
 */
std::string StringToUpper(std::string strToConvert);

/** change the string to lower case
    @param strToConvert the string to be converted
 */
std::string StringToLower(std::string strToConvert);

/** convert the number to a string
    @param n the integer number to be converted
 */
std::string NumToString(const int n);

/** convert the number to a string
    @param n the unsigned integer number to be converted
 */
std::string NumToString(const unsigned int n);

/** convert the number to a string
    @param l the long integer number to be converted
 */
std::string NumToString(const long l);

/** convert the number to a string
    @param f the float point number to be converted
    @param width format of the width of the floating number, default = 10
    @param precise format of the precise of the floating number, default = 7
 */
std::string NumToString(const float f, unsigned int width=10, unsigned int precise=7);

/** convert the number to a string
    @param d the double precise number to be converted
    @param width format of the width of the number, default = 16
    @param precise format of the precise of the number, default = 12
 */
std::string NumToString(const double d, unsigned int width=16, unsigned int precise=12);

/**
 * C++ version char* style "itoa": Convert the number to a char*
 * @param value The value to be converted.
 * @param result The char pointer to contain the result.
 * @param base The num base which has to be in the range 2 .. 16.
 * @return The converted result. If an error occured, the result is a null pointer.
 */
char* itoa( int value, char* result, int base ) ;

/**
 * C++ version std::string style "itoa": Convert the number to a string.
 * @param value The value to be converted.
 * @param base The num base which has to be in the range 2 .. 16.
 * @return The converted result. If an error occured, the result is a null pointer.
 */
std::string itoa(int value, int base);

#endif
