/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/cob_light_sim/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cob_light_sim {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"cob_light_sim");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	color_subscriber = n.subscribe("/light_controller/debug", 1, &QNode::colorCallback, this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"cob_light_sim");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	color_subscriber = n.subscribe("/light_controller/debug", 1, &QNode::colorCallback, this);
	colormulti_subscriber = n.subscribe("/light_controller/debugMulti", 1, &QNode::colorMultiCallback, this);
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(100);
	while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::colorCallback(std_msgs::ColorRGBA color)
{
  cob_light::ColorRGBAArray colors;
  colors.colors.resize(58);
  Q_EMIT colorsUpdated(colors);
}

void QNode::colorMultiCallback(cob_light::ColorRGBAArray colors)
{

}

}  // namespace cob_light_sim
