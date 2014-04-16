#include <ros/ros.h>
#include <cob_lookat_controller/cob_lookat_controller.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_lookat_controller_node");
	CobLookatController *ctc = new CobLookatController();
	
	ctc->initialize();
	ctc->run();
	
	return 0;
}
