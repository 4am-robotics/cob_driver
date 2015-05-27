#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <cob_sound/SayAction.h>
#include <cob_sound/SayText.h>

class SayAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<cob_sound::SayAction> as_;
  ros::ServiceServer srvServer_;
  ros::ServiceServer srvServer_mute_;
  ros::ServiceServer srvServer_unmute_;
  ros::Subscriber sub_;
  std::string action_name_;
  bool mute_;

public:
  diagnostic_msgs::DiagnosticArray diagnostics_;
  ros::Publisher diagnostics_pub_;
  ros::Timer diagnostics_timer_;
  ros::Publisher pubMarker_;

  SayAction(std::string name) :
    as_(nh_, name, boost::bind(&SayAction::as_cb, this, _1), false),
    action_name_(name)
  {
    as_.start();
    srvServer_ = nh_.advertiseService("say", &SayAction::service_cb, this);
    srvServer_mute_ = nh_.advertiseService("mute", &SayAction::service_cb_mute, this);
    srvServer_unmute_ = nh_.advertiseService("unmute", &SayAction::service_cb_unmute, this);
    sub_ = nh_.subscribe("say", 1000, &SayAction::topic_cb, this);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
    diagnostics_timer_ = nh_.createTimer(ros::Duration(1.0), &SayAction::timer_cb, this);
    pubMarker_ = nh_.advertise<visualization_msgs::Marker>("marker",1); //Advertise visualization marker topic
    mute_ = false;
  }

  ~SayAction(void)
  {
  }

  void as_cb(const cob_sound::SayGoalConstPtr &goal)
  {
    bool ret = say(goal->text.data);
    if (ret)
    {
        as_.setSucceeded();
    }
    else
    {
        as_.setAborted();
    }
  }

  bool service_cb(cob_sound::SayText::Request &req,
                  cob_sound::SayText::Response &res )
  {
    say(req.text);
    return true;
  }

  void topic_cb(const std_msgs::String::ConstPtr& msg)
  {
    say(msg->data.c_str());
  }

  bool service_cb_mute(std_srvs::Trigger::Request &req,
                       std_srvs::Trigger::Response &res )
  {
    mute_ = true;
    res.success = true;
    return true;
  }

  bool service_cb_unmute(std_srvs::Trigger::Request &req,
                         std_srvs::Trigger::Response &res )
  {
    mute_ = false;
    res.success = true;
    return true;
  }

  bool say(std::string text)
  {
    if (mute_)
    {
      ROS_WARN("Sound is set to mute. You will hear nothing.");
      return true;
    }

    ROS_INFO("Saying: %s", text.c_str());

    publish_marker(text);
    
    std::string mode;
    std::string command;
    std::string cepstral_conf;
    nh_.param<std::string>("/sound_controller/mode",mode,"festival");
    nh_.param<std::string>("/sound_controller/cepstral_settings",cepstral_conf,"\"speech/rate=170\"");
    if (mode == "cepstral")
    {
      command = "swift -p " + cepstral_conf + " " + text;
    }
    else
    {
      command = "echo " + text + " | text2wave | aplay -q";
    }
    if (system(command.c_str()) != 0)
    {
      ROS_ERROR("Could not play sound");
      // publishing diagnotic error if output fails
      diagnostic_msgs::DiagnosticStatus status;
      status.level = 2;
      status.name = "sound";
      status.message = "command say failed to play sound using mode " + mode;
      diagnostics_.status.push_back(status);
      
      diagnostics_.header.stamp = ros::Time::now();
      diagnostics_pub_.publish(diagnostics_);
      
      diagnostics_.status.resize(0);
      return false;
    }
    return true;
  }
  
  
  void timer_cb(const ros::TimerEvent&)
  {
    diagnostic_msgs::DiagnosticStatus status;
    status.level = 0;
    status.name = "sound";
    status.hardware_id = "none";
    status.message = "sound controller running";
    diagnostics_.status.push_back(status);
    
    diagnostics_.header.stamp = ros::Time::now();
    diagnostics_pub_.publish(diagnostics_);
    
    diagnostics_.status.resize(0);
  }

  void publish_marker(std::string text)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "color";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(std::max(text.size()*0.15, 2.0));
    marker.text = text;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 1.8;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    pubMarker_.publish(marker);
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cob_sound");

  SayAction say("say");
  ROS_INFO("sound node started");
  
  ros::spin();
  return 0;
}

