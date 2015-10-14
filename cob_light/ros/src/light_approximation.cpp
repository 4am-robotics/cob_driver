#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/LaserScan.h>
#include <cob_light/LightMode.h>
#include <colorUtils.h>

#include <algorithm>

#include <boost/thread.hpp>

sensor_msgs::LaserScan scan;
boost::mutex mutex;
ros::Publisher pubLight;
cob_light::LightMode mode_msg;
static const int NUM_LEDS = 58;
color::rgba c_red;
color::rgba c_green;

float linearInterpolate(float a, float b, float t)
{
  return a * (1 - t) + b * t;
}

color::rgba interpolateColor(color::rgba start, color::rgba goal, float t)
{
  color::hsv ca;
  color::hsv cb;
  color::hsv cr;
  color::rgba a, b;
  a = start;
  b = goal;

  a.r *= a.a;
  a.g *= a.a;
  a.b *= a.a;
  b.r *= b.a;
  b.g *= b.a;
  b.b *= b.a;
  color::Color::rgb2hsv(a.r, a.g, a.b, ca.h, ca.s, ca.v);
  color::Color::rgb2hsv(b.r, b.g, b.b, cb.h, cb.s, cb.v);

  cr.h = linearInterpolate(ca.h, cb.h, t);
  cr.s = linearInterpolate(ca.s, cb.s, t);
  cr.v = linearInterpolate(ca.v, cb.v, t);

  color::rgba result;
  color::Color::hsv2rgb(cr.h, cr.s, cr.v, result.r, result.g, result.b);
  result.a = 1.0;

  return result;
}

void scan_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
    boost::mutex::scoped_lock lock(mutex);
    scan = *msg;
}

void timerCallback(const ros::TimerEvent& event)
{
    //rotate scan array
    std::vector<float> ranges = scan.ranges;
    std::rotate(ranges.begin(), ranges.begin()+ranges.size()/2, ranges.end());
    int sector_size = ranges.size() / NUM_LEDS;
    std::vector<float> sectors;
    sectors.assign(NUM_LEDS, 0.0);

    //calculate sector values
    for(int i = 0; i < NUM_LEDS; i++)
    {
        float sum = 0;
        for(int j = i*sector_size; j < (i+1)*sector_size; j++)
        {
            sum += ranges.at(j);
        }
        sectors.at(i) = sum / sector_size;
	
	ROS_INFO_STREAM("sector "<<i<<": "<<sectors.at(i));
    }

    for(int i = 0; i < NUM_LEDS; i++)
    {
        std_msgs::ColorRGBA color;
        color::rgba col;

        if(sectors.at(i) > 2.5)
            col = c_green;
        else if(sectors.at(i) < 0.3)
            col = c_red;
        else
        {
            float mean = 0;
            if(i == 0)
                mean = sectors.back()+sectors.at(i)/2.0f;
            else
                mean = (sectors.at(i)+sectors.at(i-1))/2.0f;

            float t = (mean - 0.3)/(2.5 - 0.3);
            col = interpolateColor(c_red, c_green, t);
        }
        color.a = col.a;
        color.r = col.r;
        color.g = col.g;
        color.b = col.b;
        mode_msg.colors.at(i) = color;
	mode_msg.colors.at(NUM_LEDS/2).a = 1;
	mode_msg.colors.at(NUM_LEDS/2).r = 1;
	mode_msg.colors.at(NUM_LEDS/2).g = 0;
	mode_msg.colors.at(NUM_LEDS/2).b = 0;
    }
    pubLight.publish(mode_msg);
}

int main(int argc, char** argv)
{
  // init node
  ros::init(argc, argv, "light_approximation");
  ros::NodeHandle nh;
  ros::Subscriber sub_scan = nh.subscribe("/scan_unified", 1, scan_callback);
  pubLight = nh.advertise<cob_light::LightMode>("/light_torso/light", 1);
  ros::Timer timer = nh.createTimer(ros::Duration(0.05), timerCallback);
  std_msgs::ColorRGBA color;
  mode_msg.colors.assign(NUM_LEDS, color);
  mode_msg.priority = 1;
  mode_msg.mode = 1;

  c_red.a = 1; c_red.r = 1; c_red.g = 0; c_red.b = 0;
  c_green.a = 1; c_green.r = 0; c_green.g = 1; c_green.b = 0;

  ros::spin();

  return 0;
}
