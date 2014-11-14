#include <csignal>
#include <cstdio>
#include <lms1xx.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define DEG2RAD M_PI/180.0


// void convertScanToPolar(std::vector<int> viScanRaw,
//               std::vector<ScanPolarType>& vecScanPolar )
// {
//   double dDist;
//   double dAngle, dAngleStep;
//   double dIntens;

//   dAngleStep = fabs(m_Param.dStopAngle - m_Param.dStartAngle) / double(m_Param.iNumScanPoints - 1) ;

//   for(int i=0; i<m_Param.iNumScanPoints; i++)
//   {
//     dDist = double ((viScanRaw[i] & 0x1FFF) * m_Param.dScale);

//     dAngle = m_Param.dStartAngle + i*dAngleStep;
//     dIntens = double(viScanRaw[i] & 0x2000);

//     vecScanPolar[i].dr = dDist;
//     vecScanPolar[i].da = dAngle;
//     vecScanPolar[i].di = dIntens;
//   }
// }

int main(int argc, char **argv)
{
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

  ros::init(argc, argv, "lms1xx");
  ros::NodeHandle n("~");
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);

  if(!n.hasParam("host")) ROS_WARN("Used default parameter for host");
  n.param<std::string>("host", host, "192.168.1.2");
  if(!n.hasParam("frame_id")) ROS_WARN("Used default parameter for frame_id");
  n.param<std::string>("frame_id", frame_id, "/base_laser_link");
  if(!n.hasParam("inverted")) ROS_WARN("Used default parameter for inverted");
  n.param<bool>("inverted", inverted, false);
  if(!n.hasParam("angle_resolution")) ROS_WARN("Used default parameter for resolution");
  n.param<double>("angle_resolution", resolution, 0.5);
  if(!n.hasParam("scan_frequency")) ROS_WARN("Used default parameter for frequency");
  n.param<double>("scan_frequency", frequency, 25);
  if(!n.hasParam("set_config")) ROS_WARN("Used default parameter for set_config");
  n.param<bool>("set_config", set_config, false);

  ROS_INFO("connecting to laser at : %s", host.c_str());
  ROS_INFO("using frame_id : %s", frame_id.c_str());
  ROS_INFO("inverted : %s", (inverted)?"true":"false");
  ROS_INFO("using res : %f", resolution);
  ROS_INFO("using freq : %f", frequency);
  // initialize hardware
  laser.connect(host);

  if (laser.isConnected())
  {
    ROS_INFO("Connected to laser.");

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
      ROS_ERROR("Setting angle resolution failed");
    if(cfg.scaningFrequency != (int)(frequency * 100))
      ROS_ERROR("Setting scan frequency failed");

    //init scan msg
    scan_msg.header.frame_id = frame_id;

    scan_msg.range_min = 0.01;
    scan_msg.range_max = 20.0;

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
      return 0;
    }

    scan_msg.time_increment = scan_msg.scan_time/num_values;

    scan_msg.ranges.resize(num_values);
    scan_msg.intensities.resize(num_values);

    if(not inverted)
      scan_msg.time_increment *= -1.;

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

    while (ros::ok())
    {
      ros::Time start = ros::Time::now();

      scan_msg.header.stamp = start;
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
      }

      ros::spinOnce();
    }

    laser.scanContinous(0);
    ROS_DEBUG("scanContinous false");
    laser.stopMeas();
    ROS_DEBUG("stopMeas");
    laser.disconnect();
    ROS_DEBUG("disconnect");
  }
  else
  {
    ROS_ERROR("Connection to device failed");
  }
  return 0;
}
