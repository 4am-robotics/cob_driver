#ifndef BMS_PARAMETER_H
#define BMS_PARAMETER_H

#include "diagnostic_msgs/KeyValue.h"

struct BmsParameter
{	
	unsigned int offset;
	unsigned int length;
	
	std::string name;
	bool is_signed;
	double factor;
	std::string unit;
	bool is_topic;
	
	diagnostic_msgs::KeyValue kv;	//TODO make sure it is empty initially??
	
	BmsParameter()
	: factor(1.0)
	, unit("")
	, is_topic(false)
	{}
};

#endif
