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

#ifndef _PHIDGETIKROS_H_
#define _PHIDGETIKROS_H_

#include <ros/ros.h>
#include <cob_phidgets/phidgetik.h>
#include <cob_phidgets/SetDataRate.h>
#include <cob_phidgets/SetDigitalSensor.h>
#include <cob_phidgets/SetTriggerValue.h>
#include <cob_phidgets/DigitalSensor.h>
#include <cob_phidgets/AnalogSensor.h>

#include <thread>
#include <mutex>
#include <map>

class PhidgetIKROS: public PhidgetIK
{
public:
	PhidgetIKROS(ros::NodeHandle nh, int serial_num, std::string board_name, XmlRpc::XmlRpcValue* sensor_params, SensingMode mode);
	~PhidgetIKROS();

private:
	ros::NodeHandle _nh;
	ros::Publisher _pubAnalog;
	ros::Publisher _pubDigital;
	ros::Subscriber _subDigital;
	ros::ServiceServer _srvDigitalOut;
	ros::ServiceServer _srvDataRate;
	ros::ServiceServer _srvTriggerValue;

	int _serial_num;

	struct OutputCompare
	{
		bool updated;
		int index;
		int state;
	};

	OutputCompare _outputChanged;
	std::mutex _mutex;

	std::map<int, std::string> _indexNameMapAnalog;
	std::map<std::string, int> _indexNameMapAnalogRev;
	std::map<int, std::string> _indexNameMapDigitalIn;
	std::map<std::string, int> _indexNameMapDigitalInRev;
	std::map<int, std::string> _indexNameMapDigitalOut;
	std::map<std::string, int> _indexNameMapDigitalOutRev;
	std::map<int, std::string>::iterator _indexNameMapItr;
	std::map<std::string, int>::iterator _indexNameMapRevItr;

	auto readParams(XmlRpc::XmlRpcValue* sensor_params) -> void;

	auto update() -> void;

	auto attachHandler() -> int;
	auto detachHandler() -> int;

	auto inputChangeHandler(int index, int inputState) -> int;
	auto outputChangeHandler(int index, int outputState) -> int;
	auto sensorChangeHandler(int index, int sensorValue) -> int;

	auto onDigitalOutCallback(const cob_phidgets::DigitalSensorConstPtr& msg) -> void;
	auto setDigitalOutCallback(cob_phidgets::SetDigitalSensor::Request &req,
										cob_phidgets::SetDigitalSensor::Response &res) -> bool;
	auto setDataRateCallback(cob_phidgets::SetDataRate::Request &req,
										cob_phidgets::SetDataRate::Response &res) -> bool;
	auto setTriggerValueCallback(cob_phidgets::SetTriggerValue::Request &req,
										cob_phidgets::SetTriggerValue::Response &res) -> bool;
};
#endif //_PHIDGETIK_H_
