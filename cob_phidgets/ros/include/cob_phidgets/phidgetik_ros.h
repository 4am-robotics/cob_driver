/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 

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
	std::string _board_name;

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
