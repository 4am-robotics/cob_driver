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

// standard includes
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <math.h>

// ros includes
#include <ros/ros.h>

// ros message includes
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

// serial connection includes
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>

#define SIMULATION_ENABLED

class SerialCom
{
public:
	SerialCom() :
	 _fd(-1), m_simulation(false)
	{
	}
	~SerialCom()
	{
		closePort();
	}
	int openPort(std::string devicestring, int baudrate)
	{
		if(_fd != -1) return _fd;

		ROS_DEBUG("Open Port on %s",devicestring.c_str());
		_fd = open(devicestring.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
		if(_fd != -1)
		{
			speed_t baud = getBaudFromInt(baudrate);
			fcntl(_fd, F_SETFL, 0);
			tcgetattr(_fd, &port_settings);
			port_settings.c_cflag = baud | CRTSCTS | CS8 | CLOCAL | CREAD;
			port_settings.c_iflag = IGNPAR;
			cfsetispeed(&port_settings, baudrate);
			cfsetospeed(&port_settings, baudrate);
			tcsetattr(_fd, TCSANOW, &port_settings);
			ROS_INFO("Serial connection on %s succeeded.", devicestring.c_str());
		}
		else
		{
			ROS_ERROR("Serial connection on %s failed.", devicestring.c_str());
			#ifdef SIMULATION_ENABLED
			m_simulation = true;
			ROS_INFO("Simulation mode enabled");
			#endif
		}
		return _fd;
	}
	int sendData(std::string value)
	{
		size_t wrote = -1;
		if(m_simulation)
		{
			ROS_DEBUG("Simulation Mode: Sending  [%s]",value.c_str());
		}
		else
		{
			if(_fd != -1)
			{
				wrote = write(_fd, value.c_str(), sizeof(value.c_str()));
				ROS_DEBUG("Wrote [%s] with %i bytes from %i bytes", value.c_str(), (int)wrote, sizeof(value.c_str()));
			}
			else
			{
				ROS_WARN("Can not write to serial port. Port closed!");
			}
		}
		return wrote;
	}

	int readData(std::string &value, size_t nBytes)
	{
		char buffer[32];
		size_t rec = -1;
		rec = read(_fd, buffer, nBytes);
		value = std::string(buffer);
		return rec;
	}

	bool isOpen()
	{
		return (_fd != -1);
	}

	void closePort()
	{
		if(_fd != -1)
			close(_fd);
	}

private:
	int _fd;
	struct termios port_settings;
	bool m_simulation;

	speed_t getBaudFromInt(int baud)
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
				ROS_WARN("Unsupported Baudrate [%i]. Using default [230400]", baud);
				ret=B230400;
				break;
		}
		return ret;
	}
};

class LightControl
{
	public:
		LightControl() :
		 _invertMask(0)
		{
			char *robot_env;
			robot_env = getenv("ROBOT");
			_invertMask = (std::strcmp("raw3-1",robot_env) == 0) ? 1:0;

			_nh = ros::NodeHandle("~");
			_nh.param<std::string>("devicestring",_deviceString,"/dev/ttyUSB1");
			_nh.param<int>("baudrate",_baudrate,230400);
			_nh.param<bool>("pubmarker",_bPubMarker,false);
			
			_sub = _nh.subscribe("command", 1, &LightControl::setRGB, this);

			_pubMarker = _nh.advertise<visualization_msgs::Marker>("marker",1);

			_serialCom.openPort(_deviceString, _baudrate);
				
			if(_bPubMarker)
				_timerMarker = _nh.createTimer(ros::Duration(0.5),
						&LightControl::markerCallback, this);

			//turn off leds
			setRGB(_color);
		}

		~LightControl()
		{
		}

		enum LedMode{ STATIC = 0, BREATH = 1, BREATH_COLOR = 2, FLASH = 3, SOUND = 4 };
		
		void setRGB(std_msgs::ColorRGBA color)
		{
			if(color.r <= 1.0 && color.g <=1.0 && color.b <= 1.0)
			{
				_color = color;
				//calculate rgb spektrum for spezific alpha value, because
				//led board is not supporting alpha values
				color.r *= color.a;
				color.g *= color.a;
				color.b *= color.a;
				//led board value spektrum is from 0 - 999.
				//at r@w's led strip, 0 means fully lighted and 999 light off(_invertMask)
				color.r = (fabs(_invertMask-color.r) * 999.0);
				color.g = (fabs(_invertMask-color.g) * 999.0);
				color.b = (fabs(_invertMask-color.b) * 999.0);

				_ssOut.clear();
				_ssOut.str("");
				_ssOut << (int)color.r << " " << (int)color.g << " " << (int)color.b << "\n\r";

				_serialCom.sendData(_ssOut.str());
			}
		}
		
	private:
		SerialCom _serialCom;
		std::string _deviceString;
		int _baudrate;

		std::stringstream _ssOut;
		ros::NodeHandle _nh;
		ros::Publisher _pubMarker;
		ros::Subscriber _sub;

		int _invertMask;

		bool _bPubMarker;

		ros::Timer _timerMode;
		ros::Timer _timerMarker;
		std_msgs::ColorRGBA _color;
		
		// creates and publishes an visualization marker 
		void markerCallback(const ros::TimerEvent &event)
		{
			visualization_msgs::Marker marker;
			marker.header.frame_id = "/base_link";
			marker.header.stamp = ros::Time();
			marker.ns = "color";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = 0;
			marker.pose.position.y = 0,
			marker.pose.position.z = 1.5;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
			marker.color.a = _color.a;
			marker.color.r = _color.r;
			marker.color.g = _color.g;
			marker.color.b = _color.b;
			_pubMarker.publish(marker);
		}
};

int main(int argc, char** argv)
{
	// init node
	ros::init(argc, argv, "light_controller");
	// create LightControl instance
	LightControl LightControl;

	ros::spin();

	return 0;
}
