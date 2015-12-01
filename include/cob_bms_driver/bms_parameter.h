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
	
	BmsParameter()
	: factor(1.0)
	, unit("")
	, is_topic(false)
	{}
	//TODO: write a setter to test if required members are set properly!
};

#endif
