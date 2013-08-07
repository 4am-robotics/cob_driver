#include <ros/ros.h>
#include <cob_phidgets/phidget_manager.h>
#include <cob_phidgets/phidgetik_ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_phidgets");

	int freq;
	bool event_driven;
	std::vector<std::shared_ptr<Phidget>> phidgets;
	ros::NodeHandle nodeHandle;
	ros::NodeHandle nh("~");

	nh.param<int>("frequency",freq, 30);
	nh.param<bool>("event_driver", event_driven, false);

	PhidgetManager* manager = new PhidgetManager();
	auto devices = manager->getAttachedDevices();
	delete manager;

	PhidgetIK::SensingMode sensMode;
	if(event_driven)
		sensMode = PhidgetIK::SensingMode::EVENT;
	else
		sensMode = PhidgetIK::SensingMode::POLLING;

	if(devices.size() > 0)
	{
		for(auto& device : devices)
		{
			phidgets.push_back(
				std::make_shared<PhidgetIKROS>(nodeHandle, device.serial_num, sensMode));
		}

		ros::Rate loop_rate(freq);
		while (ros::ok())
		{
			if(sensMode == PhidgetIK::SensingMode::POLLING)
			{
				for(auto& phidget : phidgets)
				{
					phidget->update();
				}
			}
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
