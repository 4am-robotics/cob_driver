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
#include <cob_light/LightMode.h>
#include <visualization_msgs/Marker.h>

// serial connection includes
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>

class SerialCom
{
public:
	SerialCom() :
	 fd(-1)
	{
	}
	~SerialCom()
	{
		closePort();
	}
	int openPort(std::string devicestring, speed_t baudrate)
	{
		ROS_DEBUG_NAMED("SerialCom","Open Port on %s",devicestring.c_str());
		fd = open(devicestring.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
		if(fd != -1)
		{
			ROS_INFO_NAMED("SerialCom","Serial connection on %s succeeded.", devicestring.c_str());
			fcntl(fd, F_SETFL, 0);
			tcgetattr(fd, &port_settings);
			cfsetispeed(&port_settings, baudrate);
			cfsetospeed(&port_settings, baudrate);
			tcsetattr(fd, TCSANOW, &port_settings);
		}
		else
			ROS_ERROR_NAMED("SerialCom","Serial connection on %s failed.", devicestring.c_str());
		return fd;
	}
	int sendData(std::string value)
	{
		size_t wrote = -1;
		if(fd != -1)
		{
			wrote = write(fd, value.c_str(), sizeof(value.c_str()));
			if(wrote != sizeof(value.c_str()))
				ROS_ERROR_NAMED("SerialCom", "Could not write all data. Left: %i", wrote);
		}
		ROS_WARN_NAMED("SerialCom","Can not write to serial port. Port closed!");
		return wrote;
	}

	bool isOpen()
	{
		return (fd != -1);
	}

	void closePort()
	{
		if(fd != -1)
			close(fd);
	}

private:
	int fd;
	struct termios port_settings;
};

class LedController
{
	public:
		LedController() :
		 _timer_inc(0.0)
		{
			_nh = ros::NodeHandle("~");
			_nh.param<std::string>("/light_controller/devicestring",_deviceString,"/dev/ttyUSB");
			_nh.param<int>("/light_controller/baudrate",_baudrate,230400);
			_nh.param<bool>("/light_controller/pubmarker",_bPubMarker,false);
			
			_pubMarker = _nh.advertise<visualization_msgs::Marker>("marker",1);

			_serialCom.openPort(_deviceString, getBaudFromInt(_baudrate));
				
			_srvServer = 
				_nh.advertiseService("/mode", &LedController::modeCallback, this);

			if(_bPubMarker)
				_timerMarker = _nh.createTimer(ros::Duration(0.5),
						&LedController::markerCallback, this);
		}
		~LedController()
		{
		}
		
		enum LedMode{ STATIC = 0, BREATH = 1, BREATH_COLOR = 2, FLASH = 3, SOUND = 4 };
		
		void setRGB(std_msgs::ColorRGBA color)
		{
			if(color.r <= 1.0 && color.g <=1.0 && color.b <= 1.0)
			{
				_color = color;
				color.r *= 999.0; color.g *= 999.0; color.b *= 999.0;
				_ssOut << (int)color.r << " " 
						<< (int)color.g << " "
						<< (int)color.b << "\n\r";
				_serialCom.sendData(_ssOut.str());
				_ssOut.clear();
				_ssOut.str("");
			}
		}
		
	private:
		SerialCom _serialCom;
		std::string _deviceString;
		int _baudrate;

		std::stringstream _ssOut;
		ros::NodeHandle _nh;
		ros::ServiceServer _srvServer;
		ros::Publisher _pubMarker;

		bool _bPubMarker;

		ros::Timer _timerMode;
		ros::Timer _timerMarker;
		float _timer_inc;
		std_msgs::ColorRGBA _color;

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
				default: ret=B230400; break;
			}
		}
		
		bool modeCallback(cob_light::LightMode::Request &req, cob_light::LightMode::Response &res)
		{
			if(_timerMode.isValid())
				_timerMode.stop();
			
			switch(req.mode)
			{
				case STATIC:
					setRGB(req.color);
				break;
				
				case BREATH:
					setRGB(req.color);
					_timerMode = _nh.createTimer(ros::Duration(0.05),
						&LedController::breathCallback, this);
				break;
				
				case FLASH:
					setRGB(req.color);
				break;
			}
			res.error_type = 0;
			res.error_msg = "";
			return true;
		}
		
		// cyclic called callback if mode is breath
		// fades leds brightness
		void breathCallback(const ros::TimerEvent &event)
		{
			//(exp(sin(_timer_inc))-1.0/M_E)*(999.0/(M_E-1.0/M_E));
			double fV = (exp(sin(_timer_inc*M_PI)-0.36787944)*425.0336050555);
			
			_timer_inc += 0.025;
			if(_timer_inc > 2)
				_timer_inc = 0;
			
			int red = _color.r * fV;
			int green = _color.g * fV;
			int blue = _color.b * fV;
			
			_ssOut << red << " " 
					<< green << " "
					<< blue << "\n\r";
			_serialCom.sendData(_ssOut.str());
			_ssOut.clear();
			_ssOut.str("");
			
		}

		void flashCallback(const ros::TimerEvent &event)
		{
			// todo: add functionality
		}

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
	// create ledcontroller instance
	LedController ledController;
	ros::spin();

	return 0;
}
