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
	std::map<int, XmlRpc::XmlRpcValue> phidget_params_map;
	std::map<int, XmlRpc::XmlRpcValue>::iterator phidget_params_map_itr;

	nh.param<int>("frequency",freq, 30);
	nh.param<bool>("event_driver", event_driven, false);

	XmlRpc::XmlRpcValue phidget_params;
	if(nh.getParam("boards",phidget_params))
	{
		for(auto& board : phidget_params)
		{
			std::string board_name = board.first;
			ROS_DEBUG("Found Board with name '%s' in params", board_name.c_str());
			if(!board.second.hasMember("serial_num"))
			{
				ROS_ERROR("Board '%s' has no serial_num param in phidget yaml config",board_name.c_str());
				continue;
			}
			XmlRpc::XmlRpcValue board_desc = board.second;
			ros::NodeHandle boards_nh(nh, "boards");
			ros::NodeHandle sensors_nh(boards_nh, board_name);
			XmlRpc::XmlRpcValue board_param;
			if(!sensors_nh.getParam("sensors", board_param))
			{
				ROS_ERROR("Board '%s' has no sensors param in phidget yaml config",board_name.c_str());
				continue;
			}
			phidget_params_map.insert(std::make_pair(board_desc["serial_num"], board_param));
		}
	}
	else
	{
		ROS_WARN("No params for the phidget boards on the Paramserver using default name/port values");
	}

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
			phidget_params_map_itr = phidget_params_map.find(device.serial_num);
			XmlRpc::XmlRpcValue *sensors_param = (phidget_params_map_itr != phidget_params_map.end()) ? &((*phidget_params_map_itr).second) : nullptr;
			phidgets.push_back(
				std::make_shared<PhidgetIKROS>(nodeHandle, device.serial_num, sensors_param, sensMode));
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
