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

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>

class Mimic
{
public:
    Mimic():
        as_mimic_(nh_, ros::this_node::getName() + "/set_mimic", boost::bind(&Mimic::as_cb_mimic_, this, _1), false),
        new_mimic_request_(false), override_tmp_folder_(true), dist_(2,10)
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

        srvServer_mimic_ = nh_.advertiseService("set_mimic", &Mimic::service_cb_mimic, this);

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
            "--no-audio"
        };
        int argc = sizeof( argv ) / sizeof( *argv );

        vlc_inst_ = libvlc_new(argc, argv);
        vlc_player_ = libvlc_media_player_new(vlc_inst_);
        libvlc_set_fullscreen(vlc_player_, 1);
        set_mimic("default", 1, 1.0, false);
        blinking_timer_ = nh_.createTimer(ros::Duration(dist_(gen_)), &Mimic::blinking_cb, this, true);
        as_mimic_.start();
        return true;
    }

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<cob_mimic::SetMimicAction> as_mimic_;
    ros::ServiceServer srvServer_mimic_;
    ros::Timer blinking_timer_;

    libvlc_instance_t* vlc_inst_;
    libvlc_media_player_t* vlc_player_;
    libvlc_media_t* vlc_media_;

    bool new_mimic_request_;
    bool override_tmp_folder_;
    boost::mutex mutex_;

    boost::random::mt19937 gen_;
    boost::random::uniform_real_distribution<> dist_;

    bool copy_mimic_files()
    {
        ROS_INFO("copying all mimic files to /tmp/mimic...");
        std::string pkg_path = ros::package::getPath("cob_mimic");
        std::string mimics_path = pkg_path + "/common";

        try{
            if(override_tmp_folder_)
            {
                if(boost::filesystem::exists("/tmp/mimic"))
                    boost::filesystem::remove_all("/tmp/mimic");
            }
        }
        catch(boost::filesystem::filesystem_error const & e)
        {
            ROS_ERROR_STREAM(std::string(e.what()));
            return false;
        }

        if(copy_dir(boost::filesystem::path(mimics_path), boost::filesystem::path("/tmp/mimic")) )
        {
            ROS_INFO("...copied all mimic files to /tmp/mimics");
            return true;
        }
        else
        {
            ROS_ERROR("...could not copy mimic files to /tmp/mimics");
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

        blinking_timer_ = nh_.createTimer(ros::Duration(dist_(gen_)), &Mimic::blinking_cb, this, true);
    }

    bool service_cb_mimic(cob_mimic::SetMimic::Request &req,
                          cob_mimic::SetMimic::Response &res )
    {
        blinking_timer_.stop();

        res.success = set_mimic(req.mimic, req.repeat, req.speed);
        res.message = "";

        blinking_timer_ = nh_.createTimer(ros::Duration(dist_(gen_)), &Mimic::blinking_cb, this, true);
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

        std::string filename = "/tmp/mimic/" + mimic + ".mp4";

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
        set_mimic("blinking", 1, 1.5);
        blinking_timer_ = nh_.createTimer(ros::Duration(dist_(gen_)), &Mimic::blinking_cb, this, true);
    }

    bool copy_dir( boost::filesystem::path const & source,
            boost::filesystem::path const & destination )
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
            if(fs::exists(destination))
            {
                ROS_INFO_STREAM("Destination directory " << destination.string() << " already exists.");
                return false;
            }
            // Create the destination directory
            if(!fs::create_directory(destination))
            {
                ROS_ERROR_STREAM( "Unable to create destination directory" << destination.string());
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
                    if( !copy_dir(current,destination / current.filename()) )
                        return false;
                }
                else
                {
                    // Found file: Copy
                    fs::copy_file(current, destination / current.filename() );
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
