// standard includes
#include <csignal>
#include <cstdio>

// ROS includes
#include "ros/ros.h"

// ROS message includes
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticArray.h>

// external includes
#include <lms1xx.h>

#define DEG2RAD M_PI/180.0


class SickLMS1xxNode
{
public:

    SickLMS1xxNode();

    bool initalize();

    void startScanner();
    void publish();
    void stopScanner();

    ros::NodeHandle nh;

private:

    bool initalizeLaser();
    bool initalizeMessage();
    void setScanDataConfig();
    void publishError(std::string error_str);

    ros::Publisher scan_pub;
    ros::Publisher diagnostic_pub;

    // laser data
    LMS1xx laser;
    scanCfg cfg;
    scanDataCfg dataCfg;
    scanData data;
    // published data
    sensor_msgs::LaserScan scan_msg;
    // parameters
    std::string host;
    std::string frame_id;
    bool inverted;
    double resolution;
    double frequency;
    bool set_config;
    double min_range;
    double max_range;
};

SickLMS1xxNode::SickLMS1xxNode()
{
    ros::NodeHandle nh;

    scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
    diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

    if(!nh.hasParam("host")) ROS_WARN("Used default parameter for host");
    nh.param<std::string>("host", host, "192.168.1.2");
    if(!nh.hasParam("frame_id")) ROS_WARN("Used default parameter for frame_id");
    nh.param<std::string>("frame_id", frame_id, "base_laser_link");
    if(!nh.hasParam("inverted")) ROS_WARN("Used default parameter for inverted");
    nh.param<bool>("inverted", inverted, false);
    if(!nh.hasParam("angle_resolution")) ROS_WARN("Used default parameter for resolution");
    nh.param<double>("angle_resolution", resolution, 0.5);
    if(!nh.hasParam("scan_frequency")) ROS_WARN("Used default parameter for frequency");
    nh.param<double>("scan_frequency", frequency, 25);
    if(!nh.hasParam("set_config")) ROS_WARN("Used default parameter for set_config");
    nh.param<bool>("set_config", set_config, false);
    if(!nh.hasParam("min_range")) ROS_WARN("Used default parameter for min_range");
    nh.param<double>("min_range", min_range, 0.01);
    if(!nh.hasParam("max_range")) ROS_WARN("Used default parameter for max_range");
    nh.param<double>("max_range", max_range, 20.0);

    ROS_INFO("connecting to laser at : %s", host.c_str());
    ROS_INFO("using frame_id : %s", frame_id.c_str());
    ROS_INFO("inverted : %s", (inverted)?"true":"false");
    ROS_INFO("using res : %f", resolution);
    ROS_INFO("using freq : %f", frequency);
}

bool SickLMS1xxNode::initalize()
{
    bool ret = false;

    if (initalizeLaser() && initalizeMessage()) {

    setScanDataConfig();
    ret = true;
    }

    return ret;
}

bool SickLMS1xxNode::initalizeLaser()
{
    bool ret = false;

    laser.connect(host);

    if (laser.isConnected()) {

    ROS_INFO("Connected to laser.");
    ret = true;

    //setup laserscanner config
    laser.login();
    cfg = laser.getScanCfg();

    if(set_config)
    {
      ROS_DEBUG("Set angle resolution to %f deg",resolution);
      cfg.angleResolution = (int)(resolution * 10000);
      ROS_DEBUG("Set scan frequency to %f hz",frequency);
      cfg.scaningFrequency = (int)(frequency * 100);

      laser.setScanCfg(cfg);
      laser.saveConfig();
    }

    cfg = laser.getScanCfg();

    if(cfg.angleResolution != (int)(resolution * 10000))
      ROS_ERROR("Setting angle resolution failed: Current angle resolution is %f.", cfg.angleResolution/10000.0);
    if(cfg.scaningFrequency != (int)(frequency * 100))
      ROS_ERROR("Setting scan frequency failed: Current scan frequency is %f.", cfg.scaningFrequency/100.0);

    } else {
      ROS_ERROR("Connection to device failed");
      publishError("Connection to device failed");
    }
    return ret;
}

bool SickLMS1xxNode::initalizeMessage()
{
    bool ret = true;

    //init scan msg
    scan_msg.header.frame_id = frame_id;

    scan_msg.range_min = min_range;
    scan_msg.range_max = max_range;

    scan_msg.scan_time = 100.0/cfg.scaningFrequency;

    scan_msg.angle_increment = (double)cfg.angleResolution/10000.0 * DEG2RAD;
    scan_msg.angle_min = (double)cfg.startAngle/10000.0 * DEG2RAD - M_PI/2;
    scan_msg.angle_max = (double)cfg.stopAngle/10000.0 * DEG2RAD - M_PI/2;

    int num_values;
    if (cfg.angleResolution == 2500)
    {
      num_values = 1081;
    }
    else if (cfg.angleResolution == 5000)
    {
      num_values = 541;
    }
    else
    {
      ROS_ERROR("Unsupported resolution");
      publishError("Unsupported resolution");
      ret = false;
    }

    scan_msg.time_increment = scan_msg.scan_time/num_values;

    scan_msg.ranges.resize(num_values);
    scan_msg.intensities.resize(num_values);

    if(not inverted)
      scan_msg.time_increment *= -1.;

    return ret;
}

void SickLMS1xxNode::setScanDataConfig()
{
    //set scandata config
    dataCfg.outputChannel = 1;
    dataCfg.remission = true;
    dataCfg.resolution = 1;
    dataCfg.encoder = 0;
    dataCfg.position = false;
    dataCfg.deviceName = false;
    dataCfg.outputInterval = 1;

    laser.setScanDataCfg(dataCfg);
    ROS_DEBUG("setScanDataCfg");

    laser.startMeas();
    ROS_DEBUG("startMeas");
}

void SickLMS1xxNode::startScanner()
{
    status_t stat;
    do // wait for ready status
    {
      stat = laser.queryStatus();
      ros::Duration(1.0).sleep();
    }
    while (stat != ready_for_measurement);

    laser.startDevice(); // Log out to properly re-enable system after config
    ROS_DEBUG("startDevice");

    laser.scanContinous(1);
    ROS_DEBUG("scanContinous true");
}

void SickLMS1xxNode::publish()
{
    scan_msg.header.stamp = ros::Time::now();
    ++scan_msg.header.seq;

    if(laser.getData(data))
    {
    for (int i = 0; i < data.dist_len1; i++)
    {
      if(not inverted) {
      scan_msg.ranges[i] = data.dist1[data.dist_len1-1-i] * 0.001;
      scan_msg.intensities[i] = data.rssi1[data.rssi_len1-1-i];
      } else {
      scan_msg.ranges[i] = data.dist1[i] * 0.001;
      scan_msg.intensities[i] = data.rssi1[i];
      }
    }
    scan_pub.publish(scan_msg);

    //Diagnostics
    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostics.status.resize(1);
    diagnostics.status[0].level = 0;
    diagnostics.status[0].name = nh.getNamespace();
    diagnostics.status[0].message = "sick scanner running";
    diagnostic_pub.publish(diagnostics);
    }
}

void SickLMS1xxNode::stopScanner()
{
    laser.scanContinous(0);
    ROS_DEBUG("scanContinous false");
    laser.stopMeas();
    ROS_DEBUG("stopMeas");
    laser.disconnect();
    ROS_DEBUG("disconnect");
}

void SickLMS1xxNode::publishError(std::string error_str)
{
    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostics.status.resize(1);
    diagnostics.status[0].level = 2;
    diagnostics.status[0].name = nh.getNamespace();
    diagnostics.status[0].message = error_str;
    diagnostic_pub.publish(diagnostics);
}


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    ros::init(argc, argv, "sick_lms1xx_node");

    SickLMS1xxNode node;

    if (!node.initalize()) {
      return 1;
    }

    node.startScanner();

    while(ros::ok())
    {
      node.publish();

      ros::spinOnce();
    }

    node.stopScanner();

    return 0;
}
