
#ifndef DISTAPPROXMODE_H
#define DISTAPPROXMODE_H

#include <mode.h>
#include <algorithm>
#include <limits>

#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/LaserScan.h>
#include <cob_light/LightMode.h>

#include <boost/thread.hpp>
#include <boost/algorithm/clamp.hpp>

class DistApproxMode : public Mode
{
public:
    DistApproxMode(size_t num_leds, int priority = 0, double freq = 5, int pulses = 0, double timeout = 0)
        :Mode(priority, freq, pulses, timeout), _timer_inc(0), _num_leds(num_leds)
    {
        ros::NodeHandle nh;
        sub_scan = nh.subscribe("/scan_unified", 1, &DistApproxMode::scan_callback, this);
        _inc = (1. / UPDATE_RATE_HZ) * _freq;
        _colors.resize(_num_leds);
        c_red.a = 1; c_red.r = 1; c_red.g = 0; c_red.b = 0;
        c_green.a = 1; c_green.r = 0; c_green.g = 1; c_green.b = 0;
        c_off.a = 0; c_off.r = 0; c_off.g = 0; c_off.b = 0;
    }

    void scan_callback(const sensor_msgs::LaserScanConstPtr& msg)
    {
        boost::mutex::scoped_lock lock(mutex);
        scan = *msg;
    }

    void execute()
    {
        if(_timer_inc >= 1.0)
        {
            //rotate scan array
            mutex.lock();
            std::vector<float> ranges = scan.ranges;
            mutex.unlock();
            float sector_size = (float)ranges.size() / (float)_num_leds;
            std::vector<float> sectors;
            sectors.assign(_num_leds, 0.0);

            //calculate sector values
            for(int i = 0; i < _num_leds; i++)
            {
                float sum = std::numeric_limits<float>::max();
                //float sum = 0;
                for(int j = i*(int)sector_size; j < (i+1)*(int)sector_size; j++)
                {
                    if(ranges.at(j) != 0){
                        sum = std::min(ranges.at(j), sum);
                    }
                }
                sectors.at(i) = sum;
            }

            for(int i = 0; i < _num_leds; i++)
            {
                color::rgba col;

                float mean = 0;
                if(i == 0)
                {
                    mean = sectors.back()+sectors.at(i)/2.0f;
                }
                else
                {
                    mean = (sectors.at(i)+sectors.at(i-1))/2.0f;
                }

                float t = (boost::algorithm::clamp(mean, 0.6, 2.0) - 0.6)/(2.0 - 0.6);
                col = color::Color::interpolateColor(c_red, c_green, t);

                _colors.at(i) = col;
            }
            _pulsed++;
            m_sigColorsReady(_colors);
            _timer_inc = 0.0;
        }
        else
            _timer_inc += _inc;
    }

    std::string getName(){ return std::string("DistApproxMode"); }

private:
    double _timer_inc;
    double _inc;
    size_t _num_leds;

    sensor_msgs::LaserScan scan;

    ros::Subscriber sub_scan;
    boost::mutex mutex;
    color::rgba c_red;
    color::rgba c_green;
    color::rgba c_off;

};

#endif
