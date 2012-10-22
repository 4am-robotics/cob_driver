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
#include <cob_lightmode/LightMode.h>
#include <visualization_msgs/Marker.h>

#include <spektrumviz.h>

#define DEFAULT_RECORD_SIZE 1024
#define DEFAULT_APPBUFFER_SIZE DEFAULT_RECORD_SIZE/2
#define DEFAULT_SAMPLING_RATE 44100
#define DEFAULT_SUBBAND_SIZE 64
#define DEFAULT_CHANNELS 2

//#define ENABLE_GUI

class LightMode
{
	public:
		LightMode() :
		 _timer_inc(0.0)
		{
			_nh = ros::NodeHandle("~");
			_srvServer = 
				_nh.advertiseService("mode", &LightMode::modeCallback, this);

			_pub = _nh.advertise<std_msgs::ColorRGBA>("/light_controller/command",2);

			_beatController = new mybeat::BeatController(DEFAULT_RECORD_SIZE,DEFAULT_SAMPLING_RATE,DEFAULT_SUBBAND_SIZE, DEFAULT_CHANNELS);
			_beatController->signalProcessingDone()->connect(boost::bind(&LightMode::beatProcessDoneCallback,this));
			//_beatController->signalBeatSnare()->connect(boost::bind(&LightMode::beatSnareCallback,this));
			//_beatController->signalBeatDrum()->connect(boost::bind(&LightMode::beatDrumCallback,this));
			_beatController->start();
		}

		~LightMode()
		{
			_beatController->stop();
			delete _beatController;
		}

		mybeat::BeatController* getBeatController(){return _beatController;}
		
		enum LedMode{ STATIC = 0, BREATH = 1, BREATH_COLOR = 2, FLASH = 3, SOUND = 4 };
		
		void beatProcessDoneCallback()
		{
			_sound_magnitude = (_beatController->getBuffers().at(0)->pwr() / _beatController->getBuffers().at(0)->max_pwr());
			
		}

		void beatSnareCallback()
		{
			ROS_INFO_NAMED("LightMode","beat Snare");
		}

		void beatDrumCallback()
		{
			ROS_INFO_NAMED("LightMode","beat Drum");
		}
		
	private:
		ros::NodeHandle _nh;
		ros::ServiceServer _srvServer;
		ros::Publisher _pub;

		ros::Timer _timerMode;
		float _timer_inc;

		std_msgs::ColorRGBA _color;

		mybeat::BeatController *_beatController;
		float _sound_magnitude;

		
		bool modeCallback(cob_lightmode::LightMode::Request &req, cob_lightmode::LightMode::Response &res)
		{
			if(_timerMode.isValid())
				ROS_INFO("Stopping timer");
				_timerMode.stop();

			if(_beatController->getEnabled())
			{
				_beatController->stop();
				ROS_INFO("Stopping _beatController");
			}
			
			switch(req.mode)
			{
				case STATIC:
					ROS_INFO("Set Mode to Static");
					_color = req.color;
					_pub.publish(_color);
				break;
				
				case BREATH:
					ROS_INFO("Set Mode to Breath");
					_color = req.color;
					_timerMode = _nh.createTimer(ros::Duration(0.04),
						&LightMode::breathCallback, this);
				break;
				
				case BREATH_COLOR:
					ROS_INFO("Set Mode to BreathColor");
					_color = req.color;
					_timerMode = _nh.createTimer(ros::Duration(0.04),
						&LightMode::breathCallback, this);
				break;

				case FLASH:
					ROS_INFO("Set Mode to Flash");
					_color = req.color;
				break;

				case SOUND:
					ROS_INFO("Set Mode to Sound");
					_color = req.color;
					_beatController->start();
					ROS_INFO("beatController started");
					_timerMode = _nh.createTimer(ros::Duration(0.05),
						&LightMode::soundCallback, this);
				break;

				default:
					ROS_WARN_NAMED("LightMode","Unsupported Mode: %d", req.mode);
			}
			res.error_type = 0;
			res.error_msg = "";
			return true;
		}
		
		// cyclic called callback if mode is breath
		// fades leds brightness
		void breathCallback(const ros::TimerEvent &event)
		{
			//double fV = (exp(sin(_timer_inc))-1.0/M_E)*(1.000/(M_E-1.0/M_E));
			double fV = (exp(sin(_timer_inc))-0.36787944)*0.42545906411;
			
			_timer_inc += 0.025;
			if(_timer_inc >= M_PI*2)
			 	_timer_inc = 0.0;

			std_msgs::ColorRGBA col;
			col.r = _color.r;
			col.g = _color.g;
			col.b = _color.b;
			col.a = fV;
			
			_pub.publish(col);			
		}

		void flashCallback(const ros::TimerEvent &event)
		{
			// todo: add functionality
		}

		void soundCallback(const ros::TimerEvent &event)
		{
			std_msgs::ColorRGBA col;
			col.r = _color.r;
			col.g = _color.g;
			col.b = _color.b;
			if(_sound_magnitude > 1.0)
				_sound_magnitude = 1.0;
			col.a = _sound_magnitude;

			_pub.publish(col);
		}
};

int main(int argc, char** argv)
{
	#ifdef ENABLE_GUI
	if(!Glib::thread_supported())
		Glib::thread_init(); 
	#endif
	// init node
	ros::init(argc, argv, "led_mode");
	// create LightMode instance
	LightMode LightMode;

	#ifdef ENABLE_GUI
	Gtk::Main kit(argc, argv);

 	Gtk::Window win;
 	win.set_title("SpektrumViz");

 	SpektrumViz viz(LightMode.getBeatController());

 	win.add(viz);
 	viz.show();
 	win.show_all();
 	win.set_size_request(860,140);
 	#endif

	ros::Rate r(30); //Cycle-Rate: Frequency of publishing EMStopStates
	while(ros::ok())
	{     
		#ifdef ENABLE_GUI   
		while( Gtk::Main::events_pending() )
			Gtk::Main::iteration();
		#endif
		ros::spinOnce();
		r.sleep();
	}
	//ros::spin();

	return 0;
}
