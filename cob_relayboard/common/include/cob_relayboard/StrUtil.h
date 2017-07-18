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
