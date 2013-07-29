#include <cob_phidgets/sensors.h>


AnalogInSensor::AnalogInSensor(const int hw__hwIndex, const std::string &sensor_name, CPhidgetInterfaceKitHandle* iKitHandle)
	:Sensor(hw__hwIndex, sensor_name, iKitHandle)
{
}

auto AnalogInSensor::setDataRate(int ms) -> int 
{
	_dataRateMs = ms;
	return CPhidgetInterfaceKit_setDataRate(*_iKitHandle, _hwIndex, _dataRateMs);
}
auto AnalogInSensor::getDataRate() -> int 
{
	if(_dataRateMs == -1)
		CPhidgetInterfaceKit_getDataRate(*_iKitHandle, _hwIndex, &_dataRateMs);

	return _dataRateMs;
}

auto AnalogInSensor::getDataRateMax() -> int 
{
	if(_dataRateMsMax == -1)
		CPhidgetInterfaceKit_getDataRateMax(*_iKitHandle, _hwIndex, &_dataRateMsMax);

	return _dataRateMsMax;
}
auto AnalogInSensor::getDataRateMin() -> int 
{
	if(_dataRateMsMax == -1)
		CPhidgetInterfaceKit_getDataRateMin(*_iKitHandle, _hwIndex, &_dataRateMsMin);

	return _dataRateMsMin;
}

auto AnalogInSensor::setSensorChangeTrigger(int trigger) -> int
{
	_sensorChangeTrigger = trigger;
	return CPhidgetInterfaceKit_setSensorChangeTrigger(*_iKitHandle, _hwIndex, _sensorChangeTrigger);
}
auto AnalogInSensor::getSensorChangeTrigger() -> int
{
	if(_sensorChangeTrigger == -1)
		CPhidgetInterfaceKit_getSensorChangeTrigger(*_iKitHandle, _hwIndex, &_sensorChangeTrigger);
	return _sensorChangeTrigger;
}

auto AnalogInSensor::getSensorRawValue() -> int
{
	CPhidgetInterfaceKit_getSensorRawValue(*_iKitHandle, _hwIndex, &_sensorRawValue);
	return _sensorRawValue;
}
auto AnalogInSensor::getSensorValue() -> int
{
	CPhidgetInterfaceKit_getSensorRawValue(*_iKitHandle, _hwIndex, &_value);
	return _value;
}


DigitalInSensor::DigitalInSensor(const int hw__hwIndex, const std::string &sensor_name, CPhidgetInterfaceKitHandle* iKitHandle)
	:Sensor(hw__hwIndex, sensor_name, iKitHandle)
{
}

auto DigitalInSensor::getInputState() -> int
{
	CPhidgetInterfaceKit_getInputState(*_iKitHandle, _hwIndex, &_value);
	return _value;
}

DigitalOutSensor::DigitalOutSensor(const int hw__hwIndex, const std::string &sensor_name, CPhidgetInterfaceKitHandle* iKitHandle)
	:Sensor(hw__hwIndex, sensor_name, iKitHandle)
{
}

auto DigitalOutSensor::setOutputState(int state) -> int
{
	_value = state;
	return CPhidgetInterfaceKit_setOutputState(*_iKitHandle,	_hwIndex, _value);
}

auto DigitalOutSensor::getOutputState() -> int
{
	CPhidgetInterfaceKit_getOutputState(*_iKitHandle, _hwIndex,	&_value);
	return _value;
}