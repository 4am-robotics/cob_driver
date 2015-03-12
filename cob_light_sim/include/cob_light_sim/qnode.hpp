/**
 * @file /include/cob_light_sim/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cob_light_sim_QNODE_HPP_
#define cob_light_sim_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/ColorRGBA.h>
#include <cob_light/ColorRGBAArray.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cob_light_sim {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

Q_SIGNALS:
	void colorsUpdated(cob_light::ColorRGBAArray colors);
  void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
	ros::Subscriber color_subscriber;
	ros::Subscriber colormulti_subscriber;

	void colorCallback(std_msgs::ColorRGBA color);
	void colorMultiCallback(cob_light::ColorRGBAArray colors);
};

}  // namespace cob_light_sim

#endif /* cob_light_sim_QNODE_HPP_ */
