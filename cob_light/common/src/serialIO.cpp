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
 * ROS stack name: cob_driver
 * ROS package name: cob_light
 * Description: Switch robots led color by sending data to
 * the led-µC over serial connection.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Benjamin Maidel, email:benjamin.maidel@ipa.fraunhofer.de
 * Supervised by: Benjamin Maidel, email:benjamin.maidel@ipa.fraunhofer.de
 *
 * Date of creation: August 2012
 * ToDo:
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

#include "serialIO.h"

SerialIO::SerialIO() :
	 _fd(-1)
{
}

SerialIO::~SerialIO()
{
	closePort();
}

// Open Serial Port
int SerialIO::openPort(std::string devicestring, int baudrate)
{
	if(_fd != -1) return _fd;

	_fd = open(devicestring.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if(_fd != -1)
	{
		speed_t baud = getBaudFromInt(baudrate);
		fcntl(_fd, F_SETFL, 0);
		tcgetattr(_fd, &port_settings);
		port_settings.c_cflag &= ~PARENB;
		port_settings.c_cflag &= ~CSTOPB;
		port_settings.c_cflag &= ~CSIZE;
		port_settings.c_cflag = baud | CS8 | CLOCAL | CREAD;
		port_settings.c_iflag = IGNPAR;
		tcsetattr(_fd, TCSANOW, &port_settings);
	}
	return _fd;
}

// Send Data to Serial Port
int SerialIO::sendData(std::string value)
{
	int wrote = -1;
	if(_fd != -1)
		wrote = write(_fd, value.c_str(), value.length());
	return wrote;
}

// Read Data from Serial Port
int SerialIO::readData(std::string &value, size_t nBytes)
{
	char buffer[32];
	size_t rec = -1;
	rec = read(_fd, buffer, nBytes);
	value = std::string(buffer);
	return rec;
}

// Check if Serial Port is opened
bool SerialIO::isOpen()
{
	return (_fd != -1);
}

// Close Serial Port
void SerialIO::closePort()
{
	if(_fd != -1)
		close(_fd);
}


speed_t SerialIO::getBaudFromInt(int baud)
{
	speed_t ret;
	switch(baud)
	{
		case 0: 	ret=B0;		break;
		case 50: 	ret=B50; 	break;
		case 75: 	ret=B75; 	break;
		case 110: 	ret=B110;	break;
		case 134: 	ret=B134;	break;
		case 150: 	ret=B150;	break;
		case 200: 	ret=B200;	break;
		case 300: 	ret=B300;	break;
		case 1200: 	ret=B1200; 	break;
		case 1800: 	ret=B1800; 	break;
		case 2400: 	ret=B2400; 	break;
		case 4800: 	ret=B4800; 	break;
		case 9600: 	ret=B9600; 	break;
		case 19200: ret=B19200; break;
		case 38400: ret=B38400;	break;
		case 57600: ret=B57600; break;
		case 115200: ret=B115200; break;
		case 230400: ret=B230400; break;
		default:
			ret=B230400;
			break;
	}
	return ret;
}