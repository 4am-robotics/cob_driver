/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_driver
 * \note
 *   ROS package name: cob_phidgets
 *
 * \author
 *   Author: Benjamin Maidel, mail: bnm@ipa.fhg.de
 * \author
 *   Supervised by:
 *
 * \date Date of creation: 29.07.2013
 *
 * \brief
 *   this package integrates phidget boards into the ros system
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <ros/ros.h>
#include <cob_phidgets/phidget_manager.h>
#include <cob_phidgets/phidgetik_ros.h>

int main(int argc, char **argv)
{
	//init ros
	ros::init(argc, argv, "cob_phidgets");

	int freq;
	std::string update_mode;
	std::vector<std::shared_ptr<Phidget>> phidgets;
	ros::NodeHandle nodeHandle;
	ros::NodeHandle nh("~");
	std::map<int, std::pair<std::string, XmlRpc::XmlRpcValue> > phidget_params_map;
	std::map<int, std::pair<std::string, XmlRpc::XmlRpcValue> >::iterator phidget_params_map_itr;

	//get default params from server
	nh.param<int>("frequency",freq, 30);
	nh.param<std::string>("update_mode", update_mode, "polling");

	//get board params from server
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
			phidget_params_map.insert(std::make_pair(board_desc["serial_num"],
					std::make_pair(board_name, board_param)));
		}
	}
	else
	{
		ROS_WARN("No params for the phidget boards on the Paramserver using default name/port values");
	}

	//look for attached devices
	PhidgetManager* manager = new PhidgetManager();
	auto devices = manager->getAttachedDevices();
	delete manager;

	//set the update method
	PhidgetIK::SensingMode sensMode;
	if(update_mode == "event")
		sensMode = PhidgetIK::SensingMode::EVENT;
	else if(update_mode == "polling")
		sensMode = PhidgetIK::SensingMode::POLLING;
	else
	{
		ROS_WARN("Unknown update mode '%s' use polling instead", update_mode.c_str());
		sensMode = PhidgetIK::SensingMode::POLLING;
	}

	//if no devices attached exit
	if(devices.size() > 0)
	{
		//loop over all found devices and init them
		for(auto& device : devices)
		{
			phidget_params_map_itr = phidget_params_map.find(device.serial_num);
			XmlRpc::XmlRpcValue *sensors_param = (phidget_params_map_itr != phidget_params_map.end()) ? &((*phidget_params_map_itr).second.second) : nullptr;
			std::string name;
			if(phidget_params_map_itr != phidget_params_map.end())
				name = (*phidget_params_map_itr).second.first;
			else
			{
				std::stringstream ss; ss << device.serial_num;
				name = ss.str();
			}
			phidgets.push_back(
				std::make_shared<PhidgetIKROS>(nodeHandle, device.serial_num, name, sensors_param, sensMode));
		}

		ros::Rate loop_rate(freq);
		while (ros::ok())
		{
			//update devices if update method is polling
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
