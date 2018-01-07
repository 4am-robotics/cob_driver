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

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>

#include <cob_mimic/SetMimicAction.h>
#include <cob_mimic/SetMimicGoal.h>
#include <cob_mimic/SetMimicFeedback.h>
#include <cob_mimic/SetMimicResult.h>

#include <cob_mimic/SetMimic.h>
#include <cob_mimic/SetMimicRequest.h>
#include <cob_mimic/SetMimicResponse.h>

#include <vlc/vlc.h>
#include <unistd.h>

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/uniform_int_distribution.hpp>

class Mimic
{
public:
    Mimic():
        as_mimic_(nh_, ros::this_node::getName() + "/set_mimic", boost::bind(&Mimic::as_cb_mimic_, this, _1), false),
        new_mimic_request_(false), sim_enabled_(false), real_dist_(2,10), int_dist_(0,6)
    {
        nh_ = ros::NodeHandle("~");
    }

    ~Mimic(void)
    {
        libvlc_media_player_stop(vlc_player_);
        libvlc_media_player_release(vlc_player_);
        libvlc_release(vlc_inst_);
    }

    bool init()
    {
        if(!copy_mimic_files())
            return false;

        sim_enabled_ = nh_.param<bool>("sim", false);
        srvServer_mimic_ = nh_.advertiseService("set_mimic", &Mimic::service_cb_mimic, this);

        random_mimics_.push_back("blinking");
        random_mimics_.push_back("blinking");
        random_mimics_.push_back("blinking");
        random_mimics_.push_back("blinking_left");
        random_mimics_.push_back("blinking_right");

        int_dist_ = boost::random::uniform_int_distribution<>(0,static_cast<int>(random_mimics_.size())-1);

        char const *argv[] =
        {
            "--ignore-config",
            "--mouse-hide-timeout=0",
            "-q",
            "--no-osd",
            "-L",
            "--one-instance",
            "--playlist-enqueue",
            "--no-video-title-show",
            "--no-skip-frames",
            "--no-audio",
            "--vout=glx,none"
        };
        int argc = sizeof( argv ) / sizeof( *argv );

        vlc_inst_ = libvlc_new(argc, argv);
        vlc_player_ = libvlc_media_player_new(vlc_inst_);
        if(!sim_enabled_)
            libvlc_set_fullscreen(vlc_player_, 1);
        set_mimic("default", 1, 1.0, false);
        blinking_timer_ = nh_.createTimer(ros::Duration(real_dist_(gen_)), &Mimic::blinking_cb, this, true);
        as_mimic_.start();
        return true;
    }

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<cob_mimic::SetMimicAction> as_mimic_;
    ros::ServiceServer srvServer_mimic_;
    ros::Timer blinking_timer_;
    std::string mimic_folder_;

    libvlc_instance_t* vlc_inst_;
    libvlc_media_player_t* vlc_player_;
    libvlc_media_t* vlc_media_;

    bool sim_enabled_;
    bool new_mimic_request_;
    boost::mutex mutex_;

    boost::random::mt19937 gen_;
    boost::random::uniform_real_distribution<> real_dist_;
    boost::random::uniform_int_distribution<> int_dist_;
    std::vector<std::string> random_mimics_;

    bool copy_mimic_files()
    {
        char *lgn;
        if((lgn = getlogin()) == NULL)
        {
            lgn = getenv("USER");
            if(lgn == NULL || std::string(lgn) == "")
            {
                ROS_ERROR("unable to get user name");
                return false;
            }
        }
        std::string username(lgn);
        mimic_folder_ = "/tmp/mimic_" + username;
        ROS_INFO("copying all mimic files to %s...", mimic_folder_.c_str());
        std::string pkg_path = ros::package::getPath("cob_mimic");
        std::string mimics_path = pkg_path + "/common";

        try{
            if(boost::filesystem::exists(mimic_folder_))
            {
                boost::filesystem::remove_all(mimic_folder_);
            }
        }
        catch(boost::filesystem::filesystem_error const & e)
        {
            ROS_ERROR_STREAM(std::string(e.what()));
            return false;
        }

        if(copy_dir(boost::filesystem::path(mimics_path), boost::filesystem::path(mimic_folder_)) )
        {
            ROS_INFO("...copied all mimic files to %s", mimic_folder_.c_str());
            return true;
        }
        else
        {
            ROS_ERROR("...could not copy mimic files to %s", mimic_folder_.c_str());
            return false;
        }
    }

    void as_cb_mimic_(const cob_mimic::SetMimicGoalConstPtr &goal)
    {
        blinking_timer_.stop();

        if(set_mimic(goal->mimic, goal->repeat, goal->speed))
            as_mimic_.setSucceeded();
        else
            as_mimic_.setAborted();

        if(goal->mimic != "falling_asleep" && goal->mimic != "sleeping")
            blinking_timer_ = nh_.createTimer(ros::Duration(real_dist_(gen_)), &Mimic::blinking_cb, this, true);
    }

    bool service_cb_mimic(cob_mimic::SetMimic::Request &req,
                          cob_mimic::SetMimic::Response &res )
    {
        blinking_timer_.stop();

        res.success = set_mimic(req.mimic, req.repeat, req.speed);
        res.message = "";

        if(req.mimic != "falling_asleep" && req.mimic != "sleeping")
            blinking_timer_ = nh_.createTimer(ros::Duration(real_dist_(gen_)), &Mimic::blinking_cb, this, true);
        return true;
    }

    bool set_mimic(std::string mimic, int repeat, float speed, bool blocking=true)
    {
        bool ret = false;
        new_mimic_request_=true;
        ROS_INFO("New mimic request with: %s", mimic.c_str());
        mutex_.lock();
        new_mimic_request_=false;
        ROS_INFO("Mimic: %s (speed: %f, repeat: %d)", mimic.c_str(), speed, repeat);

        std::string filename = mimic_folder_ + "/" + mimic + ".mp4";

        // check if mimic exists
        if ( !boost::filesystem::exists(filename) )
        {
            ROS_ERROR("File not found: %s", filename.c_str());
            mutex_.unlock();
            return false;
        }

        // repeat cannot be 0
        repeat = std::max(1, repeat);

        // speed cannot be 0 or negative
        if(speed <= 0)
        {
            ROS_WARN("Mimic speed cannot be 0 or negative. Setting Speed to 1.0");
            speed = 1.0;
        }

        libvlc_media_player_set_rate(vlc_player_, speed);

        while(repeat > 0)
        {
            vlc_media_ = libvlc_media_new_path(vlc_inst_, filename.c_str());

            if (vlc_media_ != NULL)
            {
                libvlc_media_player_set_media(vlc_player_, vlc_media_);
                libvlc_media_release(vlc_media_);
                libvlc_media_player_play(vlc_player_);
                ros::Duration(0.1).sleep();
                while(blocking && (libvlc_media_player_is_playing(vlc_player_) == 1))
                {
                    ros::Duration(0.1).sleep();
                    ROS_DEBUG("still playing %s", mimic.c_str());
                    if(new_mimic_request_)
                    {
                        ROS_WARN("mimic %s preempted", mimic.c_str());
                        mutex_.unlock();
                        return false;
                    }
                }
                repeat --;
            }
        }
        mutex_.unlock();
        return true;
    }

    void blinking_cb(const ros::TimerEvent&)
    {
        int rand = int_dist_(gen_);
        set_mimic(random_mimics_[rand], 1, 1.5);
        blinking_timer_ = nh_.createTimer(ros::Duration(real_dist_(gen_)), &Mimic::blinking_cb, this, true);
    }

    bool copy_dir( boost::filesystem::path const & source,
            boost::filesystem::path const & mimic_folder )
    {
        namespace fs = boost::filesystem;
        try
        {
            // Check whether the function call is valid
            if(!fs::exists(source) || !fs::is_directory(source))
            {
                ROS_ERROR_STREAM("Source directory " << source.string() << " does not exist or is not a directory.");
                             ;
                return false;
            }
            if(fs::exists(mimic_folder))
            {
                ROS_INFO_STREAM("Destination directory " << mimic_folder.string() << " already exists.");
                return false;
            }
            // Create the mimic_folder directory
            if(!fs::create_directory(mimic_folder))
            {
                ROS_ERROR_STREAM( "Unable to create mimic_folder directory" << mimic_folder.string());
                return false;
            }
        }
        catch(fs::filesystem_error const & e)
        {
            ROS_ERROR_STREAM(std::string(e.what()));
            return false;
        }
        // Iterate through the source directory
        for(fs::directory_iterator file(source); file != fs::directory_iterator(); ++file)
        {
            try
            {
                fs::path current(file->path());
                if(fs::is_directory(current))
                {
                    // Found directory: Recursion
                    if( !copy_dir(current, mimic_folder / current.filename()) )
                        return false;
                }
                else
                {
                    // Found file: Copy
                    fs::copy_file(current, mimic_folder / current.filename() );
                }
            }
            catch(fs::filesystem_error const & e)
            {
                ROS_ERROR_STREAM(std::string(e.what()));
            }
        }
        return true;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mimic");

    Mimic mimic;
    if(!mimic.init())
    {
        ROS_ERROR("mimic init failed");
        return 1;
    }
    else
    {
        ROS_INFO("mimic node started");
        ros::spin();
        return 0;
    }
}
