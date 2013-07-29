#include <ros/ros.h>
#include <cob_phidgets/phidget_manager.h>
#include <cob_phidgets/phidgetik_ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_phidgets");
	ros::NodeHandle nh("~");

	std::vector<Phidget*> phidgets;

	PhidgetManager* manager = new PhidgetManager();
	auto devices = manager->getAttachedDevices();
	delete manager;

	if(devices.size() > 0)
	{

		for(auto& device : devices)
		{
			std::stringstream ss_path;
			ss_path << "/phidget_controller/board_";
			ss_path << device.serial_num << "/";
			phidgets.push_back(new PhidgetIKROS(ros::NodeHandle(ss_path.str().c_str()), device.serial_num));
		}

		ros::Rate loop_rate(20);
		while (ros::ok())
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	else
	{
		ROS_ERROR("Phidget Manager could not find any attached devices");
	}

	return 0;
}
