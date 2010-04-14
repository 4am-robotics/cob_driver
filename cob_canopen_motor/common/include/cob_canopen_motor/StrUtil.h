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
