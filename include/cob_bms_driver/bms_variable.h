#ifndef BMS_VARIABLE_H
#define BMS_VARIABLE_H

struct BmsVariable 
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
	//}
	
	double data;
};

#endif
