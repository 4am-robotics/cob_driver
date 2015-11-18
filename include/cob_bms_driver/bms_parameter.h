#ifndef BMS_PARAMETER_H
#define BMS_PARAMETER_H

struct BmsParameter
{	
	//struct Address 
	//{
		//int can_id;
		unsigned int offset;
		unsigned int length;
	//}
	
	//struct Metadata 
	//{
		std::string name;
		bool is_signed;
		double factor;
		std::string unit;
		bool is_topic;
	//}
	
	double data;
};

#endif
