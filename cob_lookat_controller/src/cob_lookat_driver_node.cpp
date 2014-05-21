#include <ros/ros.h>
#include <cob_lookat_controller/cob_lookat_driver.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_lookat_driver_node");
	CobLookatDriver *cld = new CobLookatDriver();
	
	cld->initialize();
	cld->run();
	
	return 0;
}
