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
 

#include <ros/ros.h>
#include <cob_phidgets/SetDataRate.h>
#include <cob_phidgets/SetDigitalSensor.h>
#include <cob_phidgets/SetTriggerValue.h>

#include <cob_phidgets/AnalogSensor.h>
#include <cob_phidgets/DigitalSensor.h>

class PhidgetIKROSSim
{
public:
	PhidgetIKROSSim()
	{
		_pubAnalog = _nh.advertise<cob_phidgets::AnalogSensor>("analog_sensors", 1);
		_pubDigital = _nh.advertise<cob_phidgets::DigitalSensor>("digital_sensors", 1);
		_subDigital = _nh.subscribe("set_digital_sensor", 1, &PhidgetIKROSSim::onDigitalOutCallback, this);

		_srvDigitalOut = _nh.advertiseService("set_digital", &PhidgetIKROSSim::setDigitalOutCallback, this);
		_srvDataRate = _nh.advertiseService("set_data_rate", &PhidgetIKROSSim::setDataRateCallback, this);
		_srvTriggerValue = _nh.advertiseService("set_trigger_value", &PhidgetIKROSSim::setTriggerValueCallback, this);
	};
	~PhidgetIKROSSim(){};
private:
	ros::NodeHandle _nh;
	ros::Publisher _pubAnalog;
	ros::Publisher _pubDigital;
	ros::Subscriber _subDigital;
	ros::ServiceServer _srvDigitalOut;
	ros::ServiceServer _srvDataRate;
	ros::ServiceServer _srvTriggerValue;

	auto onDigitalOutCallback(const cob_phidgets::DigitalSensorConstPtr& msg) -> void
	{

	}
	auto setDigitalOutCallback(cob_phidgets::SetDigitalSensor::Request &req,
							   cob_phidgets::SetDigitalSensor::Response &res) -> bool
	{
		res.state = req.state;
		res.uri = req.uri;
		return true;
	}
	auto setDataRateCallback(cob_phidgets::SetDataRate::Request &req,
							 cob_phidgets::SetDataRate::Response &res) -> bool
	{
		return true;
	}
	auto setTriggerValueCallback(cob_phidgets::SetTriggerValue::Request &req,
								 cob_phidgets::SetTriggerValue::Response &res) -> bool
	{
		return true;
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_phidgets_sim");
	PhidgetIKROSSim cob_phidgets_sim;
	ros::spin();
	return 0;
}
