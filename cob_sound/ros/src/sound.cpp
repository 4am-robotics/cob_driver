#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/String.h>
#include <cob_srvs/Trigger.h>
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

  SayAction(std::string name) :
    as_(nh_, name, boost::bind(&SayAction::as_cb, this, _1), false),
    action_name_(name)
  {
    as_.start();
    srvServer_ = nh_.advertiseService("/say", &SayAction::service_cb, this);
    srvServer_mute_ = nh_.advertiseService("mute", &SayAction::service_cb_mute, this);
    srvServer_unmute_ = nh_.advertiseService("unmute", &SayAction::service_cb_unmute, this);
    sub_ = nh_.subscribe("/say", 1000, &SayAction::topic_cb, this);
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

	bool service_cb_mute(cob_srvs::Trigger::Request &req,
					cob_srvs::Trigger::Response &res )
	{
        mute_ = true;
        res.success.data = true;
        return true;
	}

	bool service_cb_unmute(cob_srvs::Trigger::Request &req,
					cob_srvs::Trigger::Response &res )
	{
        mute_ = false;
        res.success.data = true;
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
    std::string mode;
    std::string command;
    std::string cepstral_conf;
    nh_.param<std::string>("/sound_controller/mode",mode,"festival");
    nh_.param<std::string>("/sound_controller/cepstral_settings",cepstral_conf,"\"speech/rate=170\"");
    if (mode == "cepstral")
    {
    	command = "/opt/swift/bin/swift -p " + cepstral_conf + " " + text;
    }
    else
    {
  		command = "echo " + text + " | text2wave | aplay -q";
  	}
    if (system(command.c_str()) != 0)
    {
    	ROS_ERROR("Could not play sound");
    	return false;
    }
    return true;
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cob_sound");

  SayAction say("say");
  ros::spin();

  return 0;
}

