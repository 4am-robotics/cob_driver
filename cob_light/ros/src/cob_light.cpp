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


// standard includes
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <math.h>
#include <signal.h>

// ros includes
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/xmlrpc_manager.h>

// ros message includes
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/UInt64.h>
#include <visualization_msgs/Marker.h>
#include <cob_light/ColorRGBAArray.h>
#include <cob_light/LightMode.h>
#include <cob_light/LightModes.h>
#include <cob_light/SetLightMode.h>
#include <cob_light/SetLightModeAction.h>
#include <cob_light/SetLightModeActionGoal.h>
#include <cob_light/SetLightModeActionResult.h>
#include <cob_light/StopLightMode.h>

// serial connection includes
#include <serialIO.h>

// additional includes
#include <colorUtils.h>
#include <modeExecutor.h>
#include <colorO.h>
#include <colorOMarker.h>
#include <colorOSim.h>
#include <ms35.h>
#include <stageprofi.h>

sig_atomic_t volatile gShutdownRequest = 0;

void sigIntHandler(int signal)
{
  ::gShutdownRequest = 1;
}

void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    ::gShutdownRequest = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

class LightControl
{
public:
  LightControl() :
    _invertMask(0), _topic_priority(0)
  {
  }
  bool init()
  {
    bool ret = true;
    bool invert_output;
    XmlRpc::XmlRpcValue param_list;
    std::string startup_mode;
    p_colorO = NULL;
    p_modeExecutor = NULL;
    //diagnostics
    _pubDiagnostic = _nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
    _diagnostics_timer = _nh.createTimer(ros::Duration(1.0), &LightControl::publish_diagnostics_cb, this);

    diagnostic_msgs::DiagnosticStatus status;
    status.name = ros::this_node::getName();

    //Get Parameter from Parameter Server
    _nh = ros::NodeHandle("~");
    if(!_nh.hasParam("invert_output"))
      ROS_WARN("Parameter 'invert_output' is missing. Using default Value: false");
    _nh.param<bool>("invert_output", invert_output, false);
    _invertMask = (int)invert_output;

    if(!_nh.hasParam("devicedriver"))
      ROS_WARN("Parameter 'devicedriver' is missing. Using default Value: cob_ledboard");
    _nh.param<std::string>("devicedriver",_deviceDriver,"cob_ledboard");

    if(!_nh.hasParam("devicestring"))
      ROS_WARN("Parameter 'devicestring' is missing. Using default Value: /dev/ttyLed");
    _nh.param<std::string>("devicestring",_deviceString,"/dev/ttyLed");

    if(!_nh.hasParam("baudrate"))
      ROS_WARN("Parameter 'baudrate' is missing. Using default Value: 230400");
    _nh.param<int>("baudrate",_baudrate,230400);

    if(!_nh.hasParam("pubmarker"))
      ROS_WARN("Parameter 'pubmarker' is missing. Using default Value: true");
    _nh.param<bool>("pubmarker",_bPubMarker,true);

    if(!_nh.hasParam("marker_frame"))
      ROS_WARN("Parameter 'marker_frame' is missing. Using default Value: /base_link");
    _nh.param<std::string>("marker_frame",_sMarkerFrame,"base_link");

    if(!_nh.hasParam("sim_enabled"))
      ROS_WARN("Parameter 'sim_enabled' is missing. Using default Value: false");
    _nh.param<bool>("sim_enabled", _bSimEnabled, false);

    if(!_nh.hasParam("startup_color"))
    {
      ROS_WARN("Parameter 'startup_color' is missing. Using default Value: off");
      _color.r=0;_color.g=0;_color.b=0;_color.a=0;
    }
    else
    {
      _nh.getParam("startup_color", param_list);
      ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(param_list.size() == 4);

      _color.r = static_cast<double>(param_list[0]);
      _color.g = static_cast<double>(param_list[1]);
      _color.b = static_cast<double>(param_list[2]);
      _color.a = static_cast<double>(param_list[3]);
    }

    if(!_nh.hasParam("startup_mode"))
      ROS_WARN("Parameter 'startup_mode' is missing. Using default Value: None");
    _nh.param<std::string>("startup_mode", startup_mode, "None");

    if(!_nh.hasParam("num_leds"))
	    ROS_WARN("Parameter 'num_leds' is missing. Using default Value: 58");
 	  _nh.param<int>("num_leds", _num_leds, 58);

    int led_offset;
    _nh.param<int>("led_offset", led_offset, 0);

    //Subscribe to LightController Command Topic
    _sub = _nh.subscribe("command", 1, &LightControl::topicCallback, this);

    //Advertise light mode Service
    _srvServer = _nh.advertiseService("set_light", &LightControl::serviceCallback, this);

    //Advertise stop mode service
    _srvStopMode = _nh.advertiseService("stop_mode", &LightControl::stopMode, this);

    //Start light mode Action Server
    _as = new ActionServer(_nh, "set_light", boost::bind(&LightControl::actionCallback, this, _1), false);
    _as->start();

    //Advertise visualization marker topic
    _pubMarker = _nh.advertise<visualization_msgs::Marker>("marker",1);

    if(!_bSimEnabled)
    {
      //open serial port
      ROS_INFO("Open Port on %s",_deviceString.c_str());
      if(_serialIO.openPort(_deviceString, _baudrate) != -1)
      {
        ROS_INFO("Serial connection on %s succeeded.", _deviceString.c_str());
        status.level = 0;
        status.message = "light controller running";

        if(_deviceDriver == "cob_ledboard")
          p_colorO = new ColorO(&_serialIO);
        else if(_deviceDriver == "ms-35")
          p_colorO = new MS35(&_serialIO);
        else if(_deviceDriver == "stageprofi")
          p_colorO = new StageProfi(&_serialIO, _num_leds, led_offset);
        else
        {
          ROS_ERROR_STREAM("Unsupported devicedriver ["<<_deviceDriver<<"], falling back to sim mode");
          p_colorO = new ColorOSim(&_nh);
          status.level = 2;
          status.message = "Unsupported devicedriver. Running in simulation mode";
        }
        p_colorO->setMask(_invertMask);
        if(!p_colorO->init())
        {
          status.level = 3;
          status.message = "Initializing connection to driver failed";
          ROS_ERROR("Initializing connection to driver failed. Exiting");
          ret = false;
        }
      }
      else
      {
        ROS_WARN("Serial connection on %s failed.", _deviceString.c_str());
        ROS_WARN("Simulation mode enabled");
        p_colorO = new ColorOSim(&_nh);
        p_colorO->setNumLeds(_num_leds);

        status.level = 2;
        status.message = "Serial connection failed. Running in simulation mode";
      }
    }
    else
    {
      ROS_INFO("Simulation mode enabled");
      if(_deviceDriver == "markers")
      {
        p_colorO = new ColorOMarker(&_nh);
        p_colorO->setNumLeds(_num_leds);
      }
      else
      {
        p_colorO = new ColorOSim(&_nh);
        p_colorO->setNumLeds(_num_leds);
      }
      status.level = 0;
      status.message = "light controller running in simulation";
    }

    _diagnostics.status.push_back(status);

    if(!ret)
      return false;

    if(_bPubMarker)
      p_colorO->signalColorSet()->connect(boost::bind(&LightControl::markerCallback, this, _1));

    p_modeExecutor = new ModeExecutor(p_colorO);

    boost::shared_ptr<Mode> mode = ModeFactory::create(startup_mode, _color);
    if(mode == NULL)
    {
      p_colorO->setColor(_color);
    }
    else
      p_modeExecutor->execute(mode);

    return true;
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

  void topicCallback(cob_light::ColorRGBAArray color)
  {
      boost::mutex::scoped_lock lock(_mutex);
      if(p_modeExecutor->getExecutingPriority() <= _topic_priority)
      {
          p_modeExecutor->pause();
          if(color.colors.size() > 0)
          {
              if(color.colors.size() > 1)
              {
                  if(color.colors.size() <= p_colorO->getNumLeds())
                  {
                    std::vector<color::rgba> colors;
                    for(size_t i=0; i<color.colors.size();i++)
                    {
                        if(color.colors[i].r <= 1.0 && color.colors[i].g <= 1.0 && color.colors[i].b <= 1.0)
                        {
                          _color.a = color.colors[i].a;
                          _color.r = color.colors[i].r;
                          _color.g = color.colors[i].g;
                          _color.b = color.colors[i].b;
                          colors.push_back(_color);
                        }
                        else
                          ROS_ERROR("Unsupported Color format. rgba values range is between 0.0 - 1.0");
                    }
                    p_colorO->setColorMulti(colors);
                  }
                  else
                    ROS_ERROR_STREAM("More colors given in ColorRGBAArray ("<<color.colors.size()<<") then driver is configured ("<<p_colorO->getNumLeds()<<"). num leds: "<<_num_leds);
              }
              else
              {
                  if(color.colors[0].r <= 1.0 && color.colors[0].g <= 1.0 && color.colors[0].b <= 1.0)
                  {
                      _color.r = color.colors[0].r;
                      _color.g = color.colors[0].g;
                      _color.b = color.colors[0].b;
                      _color.a = color.colors[0].a;

                      p_colorO->setColor(_color);
                  }
                  else
                    ROS_ERROR("Unsupported Color format. rgba values range is between 0.0 - 1.0");
              }
          }
          else
            ROS_ERROR("Empty color msg received");
        }
  }

  bool serviceCallback(cob_light::SetLightMode::Request &req, cob_light::SetLightMode::Response &res)
  {
    boost::mutex::scoped_lock lock(_mutex);
    bool ret = false;

    //ROS_DEBUG("Service Callback [Mode: %i with prio: %i freq: %f timeout: %f pulses: %i ] [R: %f with G: %f B: %f A: %f]",
    //	req.mode.mode, req.mode.priority, req.mode.frequency, req.mode.timeout, req.mode.pulses,req.mode.color.r,req.mode.color.g ,req.mode.color.b,req.mode.color.a);
    if(req.mode.colors.size() > 0)
    {
        if(req.mode.colors[0].r > 1.0 || req.mode.colors[0].g > 1.0 ||
            req.mode.colors[0].b > 1.0 || req.mode.colors[0].a > 1.0)
        {
          res.active_mode = p_modeExecutor->getExecutingMode();
          res.active_priority = p_modeExecutor->getExecutingPriority();
          res.track_id = -1;
          ROS_ERROR("Unsupported Color format. rgba values range is between 0.0 - 1.0");
        }
        else if(req.mode.mode == cob_light::LightModes::NONE) //refactor this
        {
          p_modeExecutor->stop();
          _color.a = 0;
          //p_modeExecutor->execute(req.mode);
          p_colorO->setColor(_color);
          res.active_mode = p_modeExecutor->getExecutingMode();
          res.active_priority = p_modeExecutor->getExecutingPriority();
          ret = true;
        }
        else
        {
          uint64_t u_id = p_modeExecutor->execute(req.mode);
          res.active_mode = p_modeExecutor->getExecutingMode();
          res.active_priority = p_modeExecutor->getExecutingPriority();
          res.track_id = u_id;
          ret = true;
        }
    }
    return ret;
  }

  void actionCallback(const cob_light::SetLightModeGoalConstPtr &goal)
  {
    boost::mutex::scoped_lock lock(_mutex);
    cob_light::SetLightModeResult result;
    if(goal->mode.colors.size() > 0)
    {
        if(goal->mode.colors[0].r > 1.0 || goal->mode.colors[0].g > 1.0 ||
            goal->mode.colors[0].b > 1.0 || goal->mode.colors[0].a > 1.0)
        {
          result.active_mode = p_modeExecutor->getExecutingMode();
          result.active_priority = p_modeExecutor->getExecutingPriority();
          result.track_id = -1;
          _as->setAborted(result, "Unsupported Color format. rgba values range is between 0.0 - 1.0");

          ROS_ERROR("Unsupported Color format. rgba values range is between 0.0 - 1.0");
        }
        else if(goal->mode.mode == cob_light::LightModes::NONE)
        {
          p_modeExecutor->stop();
          _color.a = 0;
          p_colorO->setColor(_color);
          result.active_mode = p_modeExecutor->getExecutingMode();
          result.active_priority = p_modeExecutor->getExecutingPriority();
          result.track_id = -1;
          _as->setSucceeded(result, "Mode switched");
        }
        else
        {
          uint64_t u_id = p_modeExecutor->execute(goal->mode);
          result.active_mode = p_modeExecutor->getExecutingMode();
          result.active_priority = p_modeExecutor->getExecutingPriority();
          result.track_id = u_id;
          _as->setSucceeded(result, "Mode switched");
        }
    }
    else
        _as->setAborted(result, "No color available");
  }

  bool stopMode(cob_light::StopLightMode::Request &req, cob_light::StopLightMode::Response &res)
  {
      boost::mutex::scoped_lock lock(_mutex);
      bool ret = false;
      ret = p_modeExecutor->stop(req.track_id);
      res.active_mode = p_modeExecutor->getExecutingMode();
      res.active_priority = p_modeExecutor->getExecutingPriority();
      res.track_id = p_modeExecutor->getExecutingUId();
      ret = true; // TODO: really check if mode is stopped
      return ret;
  }

  void publish_diagnostics_cb(const ros::TimerEvent&)
  {
    _diagnostics.header.stamp = ros::Time::now();
    _pubDiagnostic.publish(_diagnostics);
  }

  void markerCallback(color::rgba color)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = _sMarkerFrame;
    marker.header.stamp = ros::Time();
    marker.ns = "color";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.5;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
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
  std::string _deviceDriver;
  std::string _deviceString;
  int _baudrate;
  int _invertMask;
  bool _bPubMarker;
  std::string _sMarkerFrame;
  bool _bSimEnabled;
  int _num_leds;

  int _topic_priority;

  ros::NodeHandle _nh;
  ros::Subscriber _sub;
  ros::Subscriber _sub_mode;
  ros::Publisher _pubMarker;
  ros::ServiceServer _srvServer;
  ros::ServiceServer _srvStopMode;

  diagnostic_msgs::DiagnosticArray _diagnostics;
  ros::Publisher _pubDiagnostic;
  ros::Timer _diagnostics_timer;

  typedef actionlib::SimpleActionServer<cob_light::SetLightModeAction> ActionServer;
  ActionServer *_as;

  color::rgba _color;

  IColorO* p_colorO;
  SerialIO _serialIO;
  ModeExecutor* p_modeExecutor;

  boost::mutex _mutex;
};

int main(int argc, char** argv)
{
  // init node
  ros::init(argc, argv, "light_controller", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigIntHandler);

  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  // create LightControl instance
  LightControl *lightControl = new LightControl();
  if(lightControl->init())
  {
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (!gShutdownRequest)
    {
      ros::Duration(0.05).sleep();
    }

    delete lightControl;

    ros::shutdown();
  }

  return 0;
}
