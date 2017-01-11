#ifndef BMS_PARAMETER_H
#define BMS_PARAMETER_H

#include <diagnostic_msgs/KeyValue.h>
#include <ros/ros.h>

struct BmsParameter
{
	unsigned int offset;
	unsigned int length;

	std::string name;
	bool is_signed;
	double factor;
	std::string unit;

        ros::Publisher publisher;

	diagnostic_msgs::KeyValue kv;

	BmsParameter()
	: factor(1.0)
	{}
};

#endif
