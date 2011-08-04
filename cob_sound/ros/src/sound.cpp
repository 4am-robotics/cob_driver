#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/String.h>
#include <cob_sound/SayAction.h>
#include <cob_sound/SayText.h>

class SayAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<cob_sound::SayAction> as_;
  ros::ServiceServer srvServer_;
  ros::Subscriber sub;
  std::string action_name_;

public:

  SayAction(std::string name) :
    as_(nh_, name, boost::bind(&SayAction::as_cb, this, _1), false),
    action_name_(name)
  {
    as_.start();
    srvServer_ = nh_.advertiseService("/say", &SayAction::service_cb, this);
    sub = nh_.subscribe("/say", 1000, &SayAction::topic_cb, this);
  }

  ~SayAction(void)
  {
  }

  void as_cb(const cob_sound::SayGoalConstPtr &goal)
  {
	say(goal->text.data);
    as_.setSucceeded();
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

  
  void say(std::string text)
  {
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
    system(command.c_str());
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cob_sound");

  SayAction say("say");
  ros::spin();

  return 0;
}

