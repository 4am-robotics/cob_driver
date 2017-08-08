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


#include "serialIO.h"
#include "sys/select.h"
#include <iostream>
#include <cstring>

#include <ros/ros.h>

SerialIO::SerialIO() :
	 _fd(-1), _device_string(""), _baudrate(9600)
{
}

SerialIO::~SerialIO()
{
	stop();
	closePort();
}

// Open Serial Port
int SerialIO::openPort(std::string devicestring, int baudrate)
{
	if(_fd != -1) return _fd;

	_device_string = devicestring;
	_baudrate = baudrate;

	speed_t baud = getBaudFromInt(baudrate);
	std::memset(&port_settings,0,sizeof(port_settings));
	port_settings.c_iflag = 0;
	port_settings.c_oflag = 0;
	port_settings.c_cflag = CS8|CREAD|CLOCAL;
	port_settings.c_lflag = 0;
	port_settings.c_cc[VMIN]=1;
	port_settings.c_cc[VTIME]=5;

	_fd=open(devicestring.c_str(), O_RDWR | O_NONBLOCK);
	cfsetospeed(&port_settings, baud);
	cfsetispeed(&port_settings, baud);

	tcsetattr(_fd, TCSANOW, &port_settings);

	return _fd;
}

// Send Data to Serial Port
int SerialIO::sendData(std::string value)
{
	boost::mutex::scoped_lock lock(_mutex);
	int wrote = -1;
	if(_fd != -1)
		wrote = write(_fd, value.c_str(), value.length());
	return wrote;
}

// Send Data to Serial Port
int SerialIO::sendData(const char* data, size_t len)
{
	boost::mutex::scoped_lock lock(_mutex);
	int wrote = -1;
	if(_fd != -1)
		wrote = write(_fd, data, len);
	return wrote;
}

// Read Data from Serial Port
int SerialIO::readData(std::string &value, size_t nBytes)
{
	boost::mutex::scoped_lock lock(_mutex);
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(_fd, &fds);
	struct timeval timeout = {0, 100000};
	char buffer[32];
	size_t rec = -1;
	if(select(_fd+1, &fds, NULL, NULL, &timeout))
	{
		rec = read(_fd, buffer, nBytes);
		value = std::string(buffer, rec);
	}

	return rec;
}

void SerialIO::start()
{
	if(_thread == NULL)
		_thread.reset(new boost::thread(&SerialIO::run, this));
}

void SerialIO::stop()
{
	if(_thread != NULL)
	{
		_thread->interrupt();
		_thread->join();
		_thread.reset();
	}
}

void SerialIO::run()
{
	ros::Rate r(maxUpdateRate);
	std::vector<ioData_t> data;
	while(true)
	{
		_oQueue.wait_pop(data);
		for(size_t i = 0; i < data.size(); i++)
			this->sendData(data[i].buf, data[i].len);
		r.sleep();
	}
}

void SerialIO::enqueueData(std::vector<ioData_t> data)
{
	_oQueue.push(data);
}

void SerialIO::enqueueData(const char* buf, size_t len)
{
	struct ioData data;
	data.buf=buf;
	data.len=len;
	std::vector<ioData_t> vec;
	vec.push_back(data);
	_oQueue.push(vec);
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
	{
		close(_fd);
		_fd = -1;
	}
}

bool SerialIO::recover()
{
  closePort();
  if(openPort(_device_string, _baudrate))
  {
    usleep(50000);
    tcflush(_fd, TCIOFLUSH);
    return true;
  }
  else
    return false;
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
