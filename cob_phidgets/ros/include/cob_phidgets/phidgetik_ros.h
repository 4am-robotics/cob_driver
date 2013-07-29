#ifndef _PHIDGETIKROS_H_
#define _PHIDGETIKROS_H_

#include <ros/ros.h>
#include <cob_phidgets/phidgetik.h>
#include <cob_phidgets/SetDataRate.h>
#include <cob_phidgets/SetDigitalSensor.h>
#include <cob_phidgets/SetTriggerValue.h>

#include <thread>
#include <mutex>

class PhidgetIKROS: public PhidgetIK
{
public:
	PhidgetIKROS(ros::NodeHandle nh, int serial_num);

private:
	ros::NodeHandle _nh;
	ros::Publisher _pubAnalog;
	ros::Publisher _pubDigital;
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

	OutputCompare _outputChanged{false, -1, 0};
	std::mutex _mutex;

	virtual auto attachHandler() override final -> int;
	virtual auto detachHandler() override final -> int;

	auto inputChangeHandler(int index, int inputState) final override -> int;
	auto outputChangeHandler(int index, int outputState) final override -> int;
	auto sensorChangeHandler(int index, int sensorValue) final override -> int;

	auto setDigitalOutCallback(cob_phidgets::SetDigitalSensor::Request &req,
										cob_phidgets::SetDigitalSensor::Response &res) -> bool;
	auto setDataRateCallback(cob_phidgets::SetDataRate::Request &req,
										cob_phidgets::SetDataRate::Response &res) -> bool;
	auto setTriggerValueCallback(cob_phidgets::SetTriggerValue::Request &req,
										cob_phidgets::SetTriggerValue::Response &res) -> bool;
};
#endif //_PHIDGETIK_H_