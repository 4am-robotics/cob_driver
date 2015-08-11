#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <cob_sound/SayAction.h>
#include <cob_sound/PlayAction.h>
#include <cob_srvs/SetString.h>

class SoundAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<cob_sound::SayAction> as_say_;
  actionlib::SimpleActionServer<cob_sound::PlayAction> as_play_;
  ros::ServiceServer srvServer_say_;
  ros::ServiceServer srvServer_play_;
  ros::ServiceServer srvServer_mute_;
  ros::ServiceServer srvServer_unmute_;
  ros::Subscriber sub_say_;
  ros::Subscriber sub_play_;
  bool mute_;

public:
  diagnostic_msgs::DiagnosticArray diagnostics_;
  ros::Publisher diagnostics_pub_;
  ros::Timer diagnostics_timer_;
  ros::Publisher pubMarker_;

  SoundAction():
    as_say_(nh_, "say", boost::bind(&SoundAction::as_cb_say_, this, _1), false),
    as_play_(nh_, "play", boost::bind(&SoundAction::as_cb_play_, this, _1), false)
  {
    as_say_.start();
    as_play_.start();
    srvServer_say_ = nh_.advertiseService("say", &SoundAction::service_cb_say, this);
    srvServer_play_ = nh_.advertiseService("play", &SoundAction::service_cb_play, this);
    srvServer_mute_ = nh_.advertiseService("mute", &SoundAction::service_cb_mute, this);
    srvServer_unmute_ = nh_.advertiseService("unmute", &SoundAction::service_cb_unmute, this);
    sub_say_ = nh_.subscribe("say", 1, &SoundAction::topic_cb_say, this);
    sub_play_ = nh_.subscribe("play", 1, &SoundAction::topic_cb_play, this);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
    diagnostics_timer_ = nh_.createTimer(ros::Duration(1.0), &SoundAction::timer_cb, this);
    pubMarker_ = nh_.advertise<visualization_msgs::Marker>("marker",1); //Advertise visualization marker topic
    mute_ = false;
  }

  ~SoundAction(void)
  {
  }

  void as_cb_say_(const cob_sound::SayGoalConstPtr &goal)
  {
    bool ret = say(goal->text);
    if (ret)
    {
        as_say_.setSucceeded();
    }
    else
    {
        as_say_.setAborted();
    }
  }

  void as_cb_play_(const cob_sound::PlayGoalConstPtr &goal)
  {
    bool ret = play(goal->filename);
    if (ret)
    {
        as_play_.setSucceeded();
    }
    else
    {
        as_play_.setAborted();
    }
  }

  bool service_cb_say(cob_srvs::SetString::Request &req,
                  cob_srvs::SetString::Response &res )
  {
    say(req.data);
    return true;
  }

  bool service_cb_play(cob_srvs::SetString::Request &req,
                  cob_srvs::SetString::Response &res )
  {
    play(req.data);
    return true;
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

  void topic_cb_say(const std_msgs::String::ConstPtr& msg)
  {
    say(msg->data.c_str());
  }

  void topic_cb_play(const std_msgs::String::ConstPtr& msg)
  {
    play(msg->data.c_str());
  }

  bool say(std::string data)
  {
    if (mute_)
    {
      ROS_WARN("Sound is set to mute. You will hear nothing.");
      return true;
    }

    ROS_INFO("Saying: %s", data.c_str());

    publish_marker(data);

    std::string mode;
    std::string command;
    std::string cepstral_conf;
    nh_.param<std::string>("/sound_controller/mode",mode,"festival");
    nh_.param<std::string>("/sound_controller/cepstral_settings",cepstral_conf,"\"speech/rate=170\"");
    if (mode == "cepstral")
    {
      command = "aoss swift -p " + cepstral_conf + " " + data;
    }
    else
    {
      command = "echo " + data + " | text2wave | aplay -q";
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

  bool play(std::string filename)
  {
    if (mute_)
    {
      ROS_WARN("Sound is set to mute. You will hear nothing.");
      return true;
    }
    std::string command;

    ROS_INFO("Playing: %s", filename.c_str());
    command = "aplay -q " + filename;

    if (system(command.c_str()) != 0)
    {
      ROS_ERROR("Could not play file %s", filename.c_str());
      // publishing diagnotic error if output fails
      diagnostic_msgs::DiagnosticStatus status;
      status.level = 2;
      status.name = "sound";
      status.message = "command play failed";
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

  void publish_marker(std::string data)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "color";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(std::max(data.size()*0.15, 2.0));
    marker.text = data;
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

  SoundAction sound;
  ROS_INFO("sound node started");

  ros::spin();
  return 0;
}

