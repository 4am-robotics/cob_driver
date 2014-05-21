#include <ros/ros.h>
#include <cob_lookat_controller/cob_frame_tracker.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_frame_node");
	CobFrameTracker *cft = new CobFrameTracker();
	
	cft->initialize();
	cft->run();
	
	return 0;
}
