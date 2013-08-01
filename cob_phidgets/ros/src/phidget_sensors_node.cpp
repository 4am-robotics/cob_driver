#include <ros/ros.h>
#include <cob_phidgets/phidget_manager.h>
#include <cob_phidgets/phidgetik_ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_phidgets");
	ros::NodeHandle nh("~");

	std::stringstream ss_path;
	ss_path << "/phidget_controller/";
	ros::NodeHandle nodeHandle(ss_path.str().c_str());
	std::vector<std::shared_ptr<Phidget>> phidgets;

	PhidgetManager* manager = new PhidgetManager();
	auto devices = manager->getAttachedDevices();
	delete manager;

	if(devices.size() > 0)
	{
		for(auto& device : devices)
		{
			phidgets.push_back(
				std::make_shared<PhidgetIKROS>(nodeHandle, device.serial_num));
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
