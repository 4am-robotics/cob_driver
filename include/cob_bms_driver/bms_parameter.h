#ifndef BMS_PARAMETER_H
#define BMS_PARAMETER_H

struct BmsParameter
{	
	unsigned int offset;
	unsigned int length;
	
	std::string name;
	bool is_signed;
	double factor;
	std::string unit;
	bool is_topic;
};

#endif
