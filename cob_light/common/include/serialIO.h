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

	void enqueueData(std::vector<ioData_t> data);

	void enqueueData(const char* data, size_t len);

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
