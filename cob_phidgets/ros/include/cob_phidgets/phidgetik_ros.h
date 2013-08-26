#ifndef _PHIDGETIKROS_H_
#define _PHIDGETIKROS_H_

#include <ros/ros.h>
#include <cob_phidgets/phidgetik.h>
#include <cob_phidgets/SetDataRate.h>
#include <cob_phidgets/SetDigitalSensor.h>
#include <cob_phidgets/SetTriggerValue.h>

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
	std::map<int, std::string> _indexNameMapDigitalIn;
	std::map<int, std::string> _indexNameMapDigitalOut;
	std::map<int, std::string>::iterator _indexNameMapItr;

	auto readParams(XmlRpc::XmlRpcValue* sensor_params) -> void;

	auto update() -> void;

	auto attachHandler() -> int;
	auto detachHandler() -> int;

	auto inputChangeHandler(int index, int inputState) -> int;
	auto outputChangeHandler(int index, int outputState) -> int;
	auto sensorChangeHandler(int index, int sensorValue) -> int;

	auto setDigitalOutCallback(cob_phidgets::SetDigitalSensor::Request &req,
										cob_phidgets::SetDigitalSensor::Response &res) -> bool;
	auto setDataRateCallback(cob_phidgets::SetDataRate::Request &req,
										cob_phidgets::SetDataRate::Response &res) -> bool;
	auto setTriggerValueCallback(cob_phidgets::SetTriggerValue::Request &req,
										cob_phidgets::SetTriggerValue::Response &res) -> bool;
};
#endif //_PHIDGETIK_H_
