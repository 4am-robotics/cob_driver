#include <cob_phidgets/phidgetik_ros.h>
#include <cob_phidgets/DigitalSensor.h>
#include <cob_phidgets/AnalogSensor.h>

PhidgetIKROS::PhidgetIKROS(ros::NodeHandle nh, int serial_num)
	:PhidgetIK(), _serial_num(serial_num), _nh(nh)
{
	_outputChanged.updated=false;
	_outputChanged.index=-1;
	_outputChanged.state=0;

	_pubAnalog = _nh.advertise<cob_phidgets::AnalogSensor>("analog_sensor", 100);
	_pubDigital = _nh.advertise<cob_phidgets::DigitalSensor>("digital_sensor", 100);

	_srvDigitalOut = _nh.advertiseService("set_digital", &PhidgetIKROS::setDigitalOutCallback, this);
	_srvDataRate = _nh.advertiseService("set_data_rate", &PhidgetIKROS::setDataRateCallback, this);
	_srvTriggerValue = _nh.advertiseService("set_trigger_value", &PhidgetIKROS::setTriggerValueCallback, this);

	if(init(_serial_num) != EPHIDGET_OK)
	{
		ROS_ERROR("Error open Phidget Board on serial %d. Message: %s",_serial_num, this->getErrorDescription(this->getError()).c_str());
	}
	if(waitForAttachment(10000) != EPHIDGET_OK)
	{
		ROS_ERROR("Error waiting for Attachment. Message: %s",this->getErrorDescription(this->getError()).c_str());
	}
}

PhidgetIKROS::~PhidgetIKROS()
{
}

auto PhidgetIKROS::inputChangeHandler(int index, int inputState) -> int
{
	ROS_DEBUG("Board %d: Digital Input %d changed to State: %d", _serial_num, index, inputState);
	cob_phidgets::DigitalSensor msg;
	msg.index = index;
	msg.state = inputState;

	_pubDigital.publish(msg);
	return 0;
}

auto PhidgetIKROS::outputChangeHandler(int index, int outputState) -> int
{
	ROS_DEBUG("Board %d: Digital Output %d changed to State: %d", _serial_num, index, outputState);
	std::lock_guard<std::mutex> lock{_mutex};
	_outputChanged.updated = true;
	_outputChanged.index = index;
	_outputChanged.state = outputState;
	return 0;
}
auto PhidgetIKROS::sensorChangeHandler(int index, int sensorValue) -> int
{
	ROS_DEBUG("Board %d: Analog Input %d changed to Value: %d", _serial_num, index, sensorValue);
	cob_phidgets::AnalogSensor msg;
	msg.index = index;
	msg.value = sensorValue;

	_pubAnalog.publish(msg);
	return 0;
}

auto PhidgetIKROS::setDigitalOutCallback(cob_phidgets::SetDigitalSensor::Request &req,
										cob_phidgets::SetDigitalSensor::Response &res) -> bool
{
	bool ret = false;
	_mutex.lock();
	_outputChanged.updated=false;
	_outputChanged.index=-1;
	_outputChanged.state=0;
	_mutex.unlock();

	this->setOutputState(req.index, req.state);

	ros::Time start = ros::Time::now();
	while((ros::Time::now().toSec() - start.toSec()) < 1.0)
	{
		_mutex.lock();
		if(_outputChanged.updated == true)
		{
			_mutex.unlock();
			break;
		}
		_mutex.unlock();

		ros::Duration(0.025).sleep();
	}
	_mutex.lock();
	res.index = _outputChanged.index;
	res.state = _outputChanged.state;
	ROS_DEBUG("Sending response: updated: %u, index: %d, state: %d",_outputChanged.updated, _outputChanged.index, _outputChanged.state);
	ret = (_outputChanged.updated && (_outputChanged.index == req.index));

	_mutex.unlock();

	return ret;
}
auto PhidgetIKROS::setDataRateCallback(cob_phidgets::SetDataRate::Request &req,
										cob_phidgets::SetDataRate::Response &res) -> bool
{
	this->setDataRate(req.index, req.data_rate);
	return true;
}
auto PhidgetIKROS::setTriggerValueCallback(cob_phidgets::SetTriggerValue::Request &req,
										cob_phidgets::SetTriggerValue::Response &res) -> bool
{
	this->setSensorChangeTrigger(req.index, req.trigger_value);
	return true;
}

auto PhidgetIKROS::attachHandler() -> int
{
	int serialNo, version, numInputs, numOutputs, millis;
	int numSensors, triggerVal, ratiometric, i;
	const char *ptr, *name;

	CPhidget_getDeviceName((CPhidgetHandle)_iKitHandle, &name);
	CPhidget_getDeviceType((CPhidgetHandle)_iKitHandle, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)_iKitHandle, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)_iKitHandle, &version);

	CPhidgetInterfaceKit_getInputCount(_iKitHandle, &numInputs);
	CPhidgetInterfaceKit_getOutputCount(_iKitHandle, &numOutputs);
	CPhidgetInterfaceKit_getSensorCount(_iKitHandle, &numSensors);
	CPhidgetInterfaceKit_getRatiometric(_iKitHandle, &ratiometric);

	ROS_INFO("%s %d attached!!", name, serialNo);

	ROS_DEBUG("%s", ptr);
	ROS_DEBUG("Serial Number: %d\tVersion: %d", serialNo, version);
	ROS_DEBUG("Num Digital Inputs: %d\tNum Digital Outputs: %d", numInputs, numOutputs);
	ROS_DEBUG("Num Sensors: %d", numSensors);
	ROS_DEBUG("Ratiometric: %d", ratiometric);

	for(i = 0; i < numSensors; i++)
	{
		CPhidgetInterfaceKit_getSensorChangeTrigger(_iKitHandle, i, &triggerVal);
		CPhidgetInterfaceKit_getDataRate(_iKitHandle, i, &millis);

		ROS_DEBUG("Sensor#: %d > Sensitivity Trigger: %d", i, triggerVal);
		ROS_DEBUG("Sensor#: %d > Data Rate: %d", i, millis);
	}

	return 0;
}

auto PhidgetIKROS::detachHandler() -> int
{
	int serial_number;
    const char *device_name;

    CPhidget_getDeviceName ((CPhidgetHandle)_iKitHandle, &device_name);
    CPhidget_getSerialNumber((CPhidgetHandle)_iKitHandle, &serial_number);
    ROS_INFO("%s Serial number %d detached!", device_name, serial_number);
    return 0;
}
