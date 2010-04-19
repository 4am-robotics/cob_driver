#include <ros/ros.h>
#include <sstream>
#include <math.h>


float convertBinaryToFloat(unsigned int iBinaryRepresentation) {
	int iSign;
	unsigned int iExponent;
	unsigned int iMantissa;
	float iNumMantissa = 0;
		
	if((iBinaryRepresentation & (1 << 31)) == 0) 
		iSign = 1;
	else
		iSign = -1;

	iExponent = ((iBinaryRepresentation >> 23) & 0xFF) - 127; //take away Bias for positive and negative exponents
	
	iMantissa = (iBinaryRepresentation & 0x7FFFFF);
	iNumMantissa = 1;
	
	for(int i=1; i<=23; i++) {
		if((iMantissa & (1 << (23-i))) > 0) {
			iNumMantissa = iNumMantissa + 1 * pow(2,-1*i);
		}
	}
	
	return iSign * pow(2,iExponent) * iNumMantissa;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  

	ROS_ERROR_STREAM("Conversion RESULT: " << convertBinaryToFloat(0x41933333));
  
  while (ros::ok())
  {
    ros::spinOnce();
  }
}

