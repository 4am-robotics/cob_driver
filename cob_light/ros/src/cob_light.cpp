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

#include <beatcontroller.h>

//gui stuff
#include <gtkmm.h>

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
		//TODO: use devicestring
		fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
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
		char buffer[32];
		size_t wrote = -1;
		size_t rec = -1;
		if(fd != -1)
		{
			wrote = write(fd, value.c_str(), sizeof(value.c_str()));
			rec = read(fd, buffer, sizeof(value.c_str()));
			if(strcmp(buffer, value.c_str()) != 0)
				ROS_ERROR_NAMED("SerialCom","Did not received the same");
			ROS_INFO_NAMED("SerialCom","Wrote [%s] with %i bytes", value.c_str(), (int)wrote);
			ROS_INFO_NAMED("SerialCom","Received [%s] with %i bytes", buffer, (int)wrote);
			if(wrote != sizeof(value.c_str()))
				ROS_ERROR_NAMED("SerialCom", "Could not write all data. Left: %i", wrote);
		}
		else
		{
			ROS_WARN_NAMED("SerialCom","Can not write to serial port. Port closed!");
		}
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
		 _timer_inc(0.0), _sound_magnitude(0.0), _invertMask(0)
		{
			char *robot_env;
			robot_env = getenv("ROBOT");
			_invertMask = (std::strcmp("raw3-1",robot_env) == 0) ? 1:0;

			_nh = ros::NodeHandle("~");
			_nh.param<std::string>("/light_controller/devicestring",_deviceString,"/dev/ttyUSB1");
			_nh.param<int>("/light_controller/baudrate",_baudrate,230400);
			_nh.param<bool>("/light_controller/pubmarker",_bPubMarker,false);
			
			_pubMarker = _nh.advertise<visualization_msgs::Marker>("marker",1);

			_serialCom.openPort(_deviceString, getBaudFromInt(_baudrate));
				
			_srvServer = 
				_nh.advertiseService("/mode", &LedController::modeCallback, this);

			if(_bPubMarker)
				_timerMarker = _nh.createTimer(ros::Duration(0.5),
						&LedController::markerCallback, this);

			_beatController = new mybeat::BeatController(2048,44100,192);
			_beatController->addCustomBeat(600);
			_beatController->addCustomBeat(12000);
			//_beatController->signalProcessingDone()->connect(boost::bind(&LedController::beatProcessDoneCallback,this));
			//_beatController->signalBeatSnare()->connect(boost::bind(&LedController::beatSnareCallback,this));
			//_beatController->signalBeatDrum()->connect(boost::bind(&LedController::beatDrumCallback,this));
			_beatController->start();
		}

		~LedController()
		{
			_beatController->stop();
			delete _beatController;
		}

		mybeat::BeatController* getBeatController(){return _beatController;}
		
		enum LedMode{ STATIC = 0, BREATH = 1, BREATH_COLOR = 2, FLASH = 3, SOUND = 4 };
		
		void beatProcessDoneCallback()
		{
			ROS_INFO_NAMED("LedController","Processing Done");
		}

		void beatSnareCallback()
		{
			ROS_INFO_NAMED("LedController","beat Snare");
		}

		void beatDrumCallback()
		{
			ROS_INFO_NAMED("LedController","beat Drum");
		}

		void setRGB(std_msgs::ColorRGBA color)
		{
			if(color.r <= 1.0 && color.g <=1.0 && color.b <= 1.0)
			{
				_color = color;
				color.r = fabs(_invertMask-color.r) * 999.0;
				color.g = fabs(_invertMask-color.g) * 999.0;
				color.b = fabs(_invertMask-color.b) * 999.0;
				_ssOut << (int)color.r << " " 
						<< (int)color.g << " "
						<< (int)color.b << "\n\r";
				ROS_INFO("setRGB: sending %s", _ssOut.str().c_str());
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

		int _invertMask;

		bool _bPubMarker;

		ros::Timer _timerMode;
		ros::Timer _timerMarker;
		float _timer_inc;
		std_msgs::ColorRGBA _color;

		mybeat::BeatController *_beatController;
		float _sound_magnitude;

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
			return ret;
		}
		
		bool modeCallback(cob_light::LightMode::Request &req, cob_light::LightMode::Response &res)
		{
			if(_timerMode.isValid())
				_timerMode.stop();

			if(_beatController->getEnabled())
			{
						_beatController->stop();
			}
			
			switch(req.mode)
			{
				case STATIC:
					ROS_INFO_NAMED("LedController","Set Mode to Static");
					setRGB(req.color);
				break;
				
				case BREATH:
					ROS_INFO_NAMED("LedController","Set Mode to Breath");
					setRGB(req.color);
					_timerMode = _nh.createTimer(ros::Duration(0.04),
						&LedController::breathCallback, this);
				break;
				
				case FLASH:
				ROS_INFO_NAMED("LedController","Set Mode to Flash");
					setRGB(req.color);
				break;

				case SOUND:
					ROS_INFO_NAMED("LedController","Set Mode to Sound");
					_beatController->start();
					_timerMode = _nh.createTimer(ros::Duration(0.05),
						&LedController::soundCallback, this);
				break;

				default:
					ROS_WARN_NAMED("LedController","Unsupported Mode: %d", req.mode);
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
			if(_timer_inc >= 2.0)
				_timer_inc = 0.0;
			
			int red = (_invertMask * 999) - fabs(_color.r * fV);
			int green = (_invertMask * 999) - fabs(_color.g * fV);
			int blue = (_invertMask * 999) - fabs(_color.b * fV);
			
			_ssOut << red << " " << green << " " << blue << "\n\r";
			ROS_INFO("breathCallback: sending %s", _ssOut.str().c_str());
			_serialCom.sendData(_ssOut.str());
			_ssOut.clear();
			_ssOut.str("");
			
		}

		void flashCallback(const ros::TimerEvent &event)
		{
			// todo: add functionality
		}

		void soundCallback(const ros::TimerEvent &event)
		{
			int red = 999 * fabs(_invertMask-(_color.r * _sound_magnitude));
			int green = 999 * fabs(_invertMask-(_color.g * _sound_magnitude));
			int blue = 999 * fabs(_invertMask-(_color.b * _sound_magnitude));

			_ssOut << red << " " << green << " " << blue << "\n\r";
			ROS_INFO("breathCallback: sending %s", _ssOut.str().c_str());
			_serialCom.sendData(_ssOut.str());
			_ssOut.clear();
			_ssOut.str("");
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

class SoundViz : public Gtk::DrawingArea
{
	public:
		SoundViz(mybeat::BeatController* beatController)
		{
			_beatController = beatController;
			signal_draw().connect(sigc::mem_fun(*this, &SoundViz::on_draw), false);
			_beatController->signalProcessingDone()->connect(boost::bind(&SoundViz::soundProcessingDone,this));
		}
		virtual ~SoundViz(){;}

	protected:
  		//Override default signal handler:
  		virtual bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
  		{
  			Gtk::Allocation allocation = get_allocation();
			const int width = allocation.get_width();
			const int height = allocation.get_height();

			// scale to unit square and translate (0, 0) to be (0.5, 0.5), i.e.
			// the center of the window
			cr->set_line_width(1);
			//cr->set_source_rgba(0.337, 0.612, 0.117, 1);   // green
			cr->set_source_rgba(0.423, 0.482, 0.545, 0.8);   // green

			// cr->move_to(0,0);
			// cr->line_to(0, height/2);

			// cr->move_to(width,0);
			// cr->line_to(width, height/2);

			for(uint16_t i=0;i<1024;i++)
			{
				//Draw the function itself
				cr->move_to((double)i/1024.0*(double)width,height);
				cr->line_to((double)i/1024.0*(double)width,(double)height- (_beatController->getFFT()->get_magnitude(i)/_beatController->getFFT()->get_magnitude_max())*(double)height);
				//cr->line_to((double)i/2048.0*(double)width,(double)height-0.4*(double)height);
				//cr->move_to((double)i/4096*width,height-_beatController->getFFT()->get_magnitude(i)/_beatController->getFFT()->get_magnitude_max()*height);
				//cr->line_to((double)i/4096*width,height);
				cr->stroke();
			}
			uint16_t bands=_beatController->getAnalyser()->getBands();
			cr->set_source_rgba(1, 0, 0, 0.8);
			cr->move_to(0,((double)height-(_beatController->getAnalyser()->getBand(0)->getAllTimeMaximumRaw()/_beatController->getFFT()->get_magnitude_max())*(double)height));
        	for(uint16_t i=1;i<bands;i++)
        	{
				cr->line_to((double)i/bands*(double)width,(double)height-(_beatController->getAnalyser()->getBand(i)->getAllTimeMaximumRaw()/_beatController->getFFT()->get_magnitude_max())*(double)height);
			}
			cr->stroke();
			return true;
  		}

  		void soundProcessingDone()
  		{
  			// force our program to redraw the entire clock.
			Glib::RefPtr<Gdk::Window> win = get_window();
			if (win)
			{
				Gdk::Rectangle r(0, 0, get_allocation().get_width(),
				        get_allocation().get_height());
				win->invalidate_rect(r, false);
			}
  		}

  		mybeat::BeatController* _beatController;
};

int main(int argc, char** argv)
{
	if(!Glib::thread_supported())
		Glib::thread_init(); 
	// init node
	ros::init(argc, argv, "light_controller");
	// create ledcontroller instance
	LedController ledController;

	Gtk::Main kit(argc, argv);

 	Gtk::Window win;
 	win.set_title("SoundViz");

 	SoundViz viz(ledController.getBeatController());

 	win.add(viz);
 	viz.show();
 	win.show_all();
 	win.set_size_request(480,120);

	ros::Rate r(20); //Cycle-Rate: Frequency of publishing EMStopStates
	while(ros::ok())
	{        
		while( Gtk::Main::events_pending() )
			Gtk::Main::iteration();
		ros::spinOnce();
		r.sleep();
	}
	ros::spin();

	return 0;
}
