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

#ifndef SERIALIO_H
#define SERIALIO_H

// serial connection includes
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>

#include <string>

#include <concurrentQueue.h>
#include <boost/thread.hpp>

typedef struct ioData{
	const char* buf;
	size_t len;
} ioData_t;

class SerialIO
{
public:
	// Constructor
	SerialIO();
	// Destructor
	~SerialIO();

	// Open Serial Port
	int openPort(std::string devicestring, int baudrate);

	// Send Data to Serial Port
	int sendData(std::string value);

	// Send Data to Serial Port
	int sendData(const char* data, size_t len);

	// Read Data from Serial Port
	int readData(std::string &value, size_t nBytes);

	bool enqueueData(std::vector<ioData_t> data);

	bool enqueueData(const char* data, size_t len);

	// Check if Serial Port is opened
	bool isOpen();

	// Close Serial Port
	void closePort();

	bool recover();

	void start();
	void stop();

private:
	//ioQueue
	ConcurrentQueue<std::vector<struct ioData> > _oQueue;

	boost::shared_ptr<boost::thread> _thread;
	boost::mutex _mutex;
	boost::condition_variable _condition;

	// filedescriptor
	int _fd;
	// serial port settings
	struct termios port_settings;
	// device string
	std::string _device_string;
	// baudrate
	int _baudrate;

	// resolve int to baudrate
	speed_t getBaudFromInt(int baud);

	static const int maxUpdateRate = 50;

	void run();
};

#endif
