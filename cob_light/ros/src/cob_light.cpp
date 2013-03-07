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
#include <actionlib/server/simple_action_server.h>

// ros message includes
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <cob_light/LightMode.h>
#include <cob_light/SetLightMode.h>
#include <cob_light/SetLightModeAction.h>
#include <cob_light/SetLightModeActionGoal.h>
#include <cob_light/SetLightModeActionResult.h>

// serial connection includes
 #include <serialIO.h>

// additional includes
#include <colorUtils.h>
#include <modeExecutor.h>
#include <colorO.h>
#include <colorOSim.h>

#define SIMULATION_ENABLED
// define this if you like to turn off the light
// when this node comes up
#define TURN_OFF_LIGHT_ON_STARTUP

class LightControl
{
	public:
		LightControl() :
		 _invertMask(0), _topic_priority(0)
		{
			p_colorO = NULL;
			p_modeExecutor = NULL;

			bool invert_output;

			_nh = ros::NodeHandle("~");
			if(!_nh.hasParam("invert_output")) ROS_WARN("Parameter 'invert_output' is missing on ParameterServer. Using default Value");
			_nh.param<bool>("invert_output", invert_output, false);
			_invertMask = (int)invert_output;

			if(!_nh.hasParam("devicestring")) ROS_WARN("Parameter 'devicestring' is missing on ParameterServer. Using default Value");
			_nh.param<std::string>("devicestring",_deviceString,"/dev/ttyLed");
			if(!_nh.hasParam("baudrate")) ROS_WARN("Parameter 'baudrate' is missing on ParameterServer. Using default Value");
			_nh.param<int>("baudrate",_baudrate,230400);
			if(!_nh.hasParam("pubmarker")) ROS_WARN("Parameter 'pubmarker' is missing on ParameterServer. Using default Value");
			_nh.param<bool>("pubmarker",_bPubMarker,false);
			
			_sub = _nh.subscribe("command", 1, &LightControl::commandCallback, this);

			_srvServer = 
				_nh.advertiseService("mode", &LightControl::modeCallback, this);

			_as = new ActionServer(_nh, "set_lightmode", boost::bind(&LightControl::actionCallback, this, _1), false);
			_as->start();

			_pubMarker = 
				_nh.advertise<visualization_msgs::Marker>("marker",1);

			//open serial port
			ROS_INFO("Open Port on %s",_deviceString.c_str());
			if(_serialIO.openPort(_deviceString, _baudrate) != -1)
			{
				ROS_INFO("Serial connection on %s succeeded.", _deviceString.c_str());
				p_colorO = new ColorO(&_serialIO);
				p_colorO->setMask(_invertMask);
			}
			else
			{
				ROS_ERROR("Serial connection on %s failed.", _deviceString.c_str());
				#ifdef SIMULATION_ENABLED
				ROS_INFO("Simulation Mode Enabled");
				p_colorO = new ColorOSim(&_nh);
				#endif
			}

			if(_bPubMarker)
				p_colorO->signalColorSet()->connect(boost::bind(&LightControl::markerCallback, this, _1));
				
			//initial turn off leds
			#ifdef TURN_OFF_LIGHT_ON_STARTUP
				_color.a=0;
				p_colorO->setColor(_color);
			#endif

			p_modeExecutor = new ModeExecutor(p_colorO);
		}

		~LightControl()
		{
			if(p_modeExecutor != NULL)
			{
				p_modeExecutor->stop();
				delete p_modeExecutor;
			}
			if(p_colorO != NULL)
			{
				delete p_colorO;
			}
		}

		void commandCallback(std_msgs::ColorRGBA color)
		{
			if(color.r <= 1.0 && color.g <=1.0 && color.b <= 1.0)
			{
				if(p_modeExecutor->getExecutingPriority() <= _topic_priority)
				{
					p_modeExecutor->stop();
					_color.r = color.r;
					_color.g = color.g;
					_color.b = color.b;
					_color.a = color.a;
					p_colorO->setColor(_color);
				}
			}
			else
				ROS_ERROR("Unsupported Color format. rgba values range is between 0.0 - 1.0");
		}

		bool modeCallback(cob_light::SetLightMode::Request &req, cob_light::SetLightMode::Response &res)
		{
			bool ret = false;

			if(req.mode.color.r > 1.0 || req.mode.color.g > 1.0 || req.mode.color.b > 1.0 || req.mode.color.a > 1.0)
			{
				res.active_mode = p_modeExecutor->getExecutingMode();
				res.active_priority = p_modeExecutor->getExecutingPriority();
				ROS_ERROR("Unsupported Color format. rgba values range is between 0.0 - 1.0");
			}
			else if(req.mode.mode == cob_light::LightMode::NONE)
			{
				p_modeExecutor->stop();
				_color.a = 0;
				p_colorO->setColor(_color);
				res.active_mode = p_modeExecutor->getExecutingMode();
				res.active_priority = p_modeExecutor->getExecutingPriority();
				ret = true;
			}
			else
			{
				p_modeExecutor->execute(req.mode);
				res.active_mode = p_modeExecutor->getExecutingMode();
				res.active_priority = p_modeExecutor->getExecutingPriority();
				ret = true;
			}
			
			return ret;
		}

		void actionCallback(const cob_light::SetLightModeGoalConstPtr &goal)
		{
			cob_light::SetLightModeResult result;
			if(goal->mode.color.r > 1.0 || goal->mode.color.g > 1.0 || goal->mode.color.b > 1.0 || goal->mode.color.a > 1.0)
			{
				result.active_mode = p_modeExecutor->getExecutingMode();
				result.active_priority = p_modeExecutor->getExecutingPriority();
				_as->setAborted(result, "Unsupported Color format. rgba values range is between 0.0 - 1.0");
				
				ROS_ERROR("Unsupported Color format. rgba values range is between 0.0 - 1.0");
			}
			else if(goal->mode.mode == cob_light::LightMode::NONE)
			{
				p_modeExecutor->stop();
				_color.a = 0;
				p_colorO->setColor(_color);

				result.active_mode = p_modeExecutor->getExecutingMode();
				result.active_priority = p_modeExecutor->getExecutingPriority();
				_as->setSucceeded(result, "Mode switched");
			}
			else
			{
				p_modeExecutor->execute(goal->mode);
				result.active_mode = p_modeExecutor->getExecutingMode();
				result.active_priority = p_modeExecutor->getExecutingPriority();
				_as->setSucceeded(result, "Mode switched");
			}
		}

		void markerCallback(color::rgba color)
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
			marker.color.a = color.a;
			marker.color.r = color.r;
			marker.color.g = color.g;
			marker.color.b = color.b;
			_pubMarker.publish(marker);
		}
		
	private:
		std::string _deviceString;
		int _baudrate;
		int _invertMask;
		bool _bPubMarker;

		int _topic_priority;

		ros::NodeHandle _nh;
		ros::Subscriber _sub;
		ros::Publisher _pubMarker;
		ros::ServiceServer _srvServer;

		typedef actionlib::SimpleActionServer<cob_light::SetLightModeAction> ActionServer;
		ActionServer *_as;

		color::rgba _color;
		
		IColorO* p_colorO;
		SerialIO _serialIO;
		ModeExecutor* p_modeExecutor;
};

int main(int argc, char** argv)
{
	// init node
	ros::init(argc, argv, "light_controller");
	// create LightControl instance
	LightControl lightControl;

	ros::spin();
	return 0;
}
