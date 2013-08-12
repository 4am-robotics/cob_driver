#include <cob_phidgets/phidgetik.h>


PhidgetIK::PhidgetIK(SensingMode mode)	: Phidget((CPhidgetHandle*) &_iKitHandle, mode), _iKitHandle(0)
{
	_last_error = CPhidgetInterfaceKit_create(&_iKitHandle);

	if (!_last_error) {
		CPhidget_set_OnAttach_Handler((CPhidgetHandle) _iKitHandle,
				PhidgetIK::attachDelegate, this);
		CPhidgetInterfaceKit_set_OnOutputChange_Handler(_iKitHandle,
				PhidgetIK::outputChangeDelegate, this);

		if(_sensMode == SensingMode::EVENT)
		{
			CPhidgetInterfaceKit_set_OnInputChange_Handler(_iKitHandle,
					PhidgetIK::inputChangeDelegate, this);
			_last_error = CPhidgetInterfaceKit_set_OnSensorChange_Handler(
					_iKitHandle, PhidgetIK::sensorChangeDelegate, this);
		}
	}
}

PhidgetIK::~PhidgetIK()
{
}

auto PhidgetIK::init(int serial_number) -> int
{
	return (_last_error = open(serial_number));
}

auto PhidgetIK::getInputCount() -> int
{
	int count = -1;

	_last_error = CPhidgetInterfaceKit_getInputCount(_iKitHandle, &count);

	return count;
}

auto PhidgetIK::getInputState(int index) -> int
{
	int state = -1;

	 _last_error = CPhidgetInterfaceKit_getInputState(_iKitHandle, index,
	 		&state);
	
	//DigitalInSensor* sensor = dynamic_cast<DigitalInSensor*>(_sensorsMap[SensorType::DIGITAL_IN][index]);

	//if(sensor)
	//	state = sensor->getInputState();

	return state;
}

auto PhidgetIK::getOutputCount() -> int
{
	int count = -1;

	_last_error = CPhidgetInterfaceKit_getOutputCount(_iKitHandle, &count);

	return count;
}

auto PhidgetIK::getOutputState(int index) -> int
{
	int state = -1;

	 _last_error = CPhidgetInterfaceKit_getOutputState(_iKitHandle, index,
	 		&state);

	//DigitalOutSensor* sensor = dynamic_cast<DigitalOutSensor*>(_sensorsMap[SensorType::DIGITAL_OUT][index]);

	//if(sensor)
	//	state = sensor->getOutputState();

	return state;
}

auto PhidgetIK::setOutputState(int index, int state) -> int
{
//	auto ret = -1;
	return (_last_error = CPhidgetInterfaceKit_setOutputState(_iKitHandle,
	 		index, state));

	//DigitalOutSensor* sensor = dynamic_cast<DigitalOutSensor*>(_sensorsMap[SensorType::DIGITAL_OUT][index]);

	//if(sensor)
	//	ret = sensor->setOutputState(state);
	//return ret;
}

auto PhidgetIK::getSensorCount() -> int
{
	int count = -1;

	_last_error = CPhidgetInterfaceKit_getSensorCount(_iKitHandle, &count);

	return count;
}

auto PhidgetIK::getSensorValue(int index) -> int
{
	int value = -1;

	 _last_error = CPhidgetInterfaceKit_getSensorValue(_iKitHandle, index, &value);

	//AnalogInSensor* sensor = dynamic_cast<AnalogInSensor*>(_sensorsMap[SensorType::ANALOG][index]);

	//if(sensor)
	//	value = sensor->getSensorValue();

	return value;
}

auto PhidgetIK::getSensorRawValue(int index) -> int
{
	int value = -1;

	 _last_error = CPhidgetInterfaceKit_getSensorRawValue(_iKitHandle, index, &value);

	//AnalogInSensor* sensor = dynamic_cast<AnalogInSensor*>(_sensorsMap[SensorType::ANALOG][index]);

	//if(sensor)
	//	value = sensor->getSensorRawValue();

	return value;
}

auto PhidgetIK::getSensorChangeTrigger(int index) -> int
{
	int trigger = -1;

	 _last_error = CPhidgetInterfaceKit_getSensorChangeTrigger(_iKitHandle,	index, &trigger);

	//AnalogInSensor* sensor = dynamic_cast<AnalogInSensor*>(_sensorsMap[SensorType::ANALOG][index]);

	//if(sensor)
	//	trigger = sensor->getSensorChangeTrigger();

	return trigger;
}

auto PhidgetIK::setSensorChangeTrigger(int index, int trigger) -> int
{
	auto ret = -1;
	return (_last_error = CPhidgetInterfaceKit_setSensorChangeTrigger(_iKitHandle, index, trigger));

	//AnalogInSensor* sensor = dynamic_cast<AnalogInSensor*>(_sensorsMap[SensorType::ANALOG][index]);

	//if(sensor)
	//	ret = sensor->setSensorChangeTrigger(trigger);
	return ret;
}

auto PhidgetIK::getRatiometric() -> int
{
	int ratiometric = -1;

	_last_error = CPhidgetInterfaceKit_getRatiometric(_iKitHandle,
			&ratiometric);

	return ratiometric;
}

auto PhidgetIK::setRatiometric(int ratiometric) -> int
{
	return (_last_error = CPhidgetInterfaceKit_setRatiometric(_iKitHandle,
			ratiometric));
}

auto PhidgetIK::getDataRate(int index) -> int
{
	int datarate = -1;

	_last_error = CPhidgetInterfaceKit_getDataRate(_iKitHandle, index, &datarate);

	//AnalogInSensor* sensor = dynamic_cast<AnalogInSensor*>(_sensorsMap[SensorType::ANALOG][index]);

	//if(sensor)
	//	datarate = sensor->getDataRateMax();

	return datarate;
}

auto PhidgetIK::setDataRate(int index, int datarate) -> int
{
	//int ret = -1;

	return (_last_error = CPhidgetInterfaceKit_setDataRate(_iKitHandle,	index, datarate));

	//AnalogInSensor* sensor = dynamic_cast<AnalogInSensor*>(_sensorsMap[SensorType::ANALOG][index]);

	//if(sensor)
	//	ret = sensor->setDataRate(datarate);

	//return ret;
}

auto PhidgetIK::getDataRateMax(int index) -> int
{
	int max = -1;

	_last_error = CPhidgetInterfaceKit_getDataRateMax(_iKitHandle, index, &max);

	// AnalogInSensor* sensor = dynamic_cast<AnalogInSensor*>(_sensorsMap[SensorType::ANALOG][index]);

	// if(sensor)
	// 	max = sensor->getDataRateMax();

	return max;
}

auto PhidgetIK::getDataRateMin(int index) -> int
{
	int min = -1;

	_last_error = CPhidgetInterfaceKit_getDataRateMin(_iKitHandle, index, &min);

	// AnalogInSensor* sensor = dynamic_cast<AnalogInSensor*>(_sensorsMap[SensorType::ANALOG][index]);

	// if(sensor)
	// 	min = sensor->getDataRateMin();

	return min;
}

auto PhidgetIK::getError() -> int
{
	return _last_error;
}

auto PhidgetIK::attachHandler() -> int
{
	int serialNo, version, numInputs, numOutputs;
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

	printf("%s %d attached!\n", name, serialNo);

	printf("%s", ptr);
	printf("Serial Number: %d\tVersion: %d\n", serialNo, version);
	printf("Num Digital Inputs: %d\tNum Digital Outputs: %d\n", numInputs, numOutputs);
	printf("Num Sensors: %d\n", numSensors);
	printf("Ratiometric: %d\n", ratiometric);

	for(i = 0; i < numSensors; i++)
	{
		CPhidgetInterfaceKit_getSensorChangeTrigger (_iKitHandle, i, &triggerVal);

		printf("Sensor#: %d > Sensitivity Trigger: %d\n", i, triggerVal);
	}

	return 0;
}

auto PhidgetIK::detachHandler() -> int
{
	int serial_number;
    const char *device_name;

    CPhidget_getDeviceName ((CPhidgetHandle)_iKitHandle, &device_name);
    CPhidget_getSerialNumber((CPhidgetHandle)_iKitHandle, &serial_number);
    printf("%s Serial number %d detached!\n", device_name, serial_number);
    return 0;
}

auto PhidgetIK::inputChangeHandler(int index, int inputState) -> int
{
	return 0;//_sensorsMap[SensorType::DIGITAL_IN][index]->update(inputState);
}

auto PhidgetIK::outputChangeHandler(int index, int outputState) -> int
{
	return 0;//_sensorsMap[SensorType::DIGITAL_OUT][index]->update(outputState);
}

auto PhidgetIK::sensorChangeHandler(int index, int sensorValue) -> int
{
	return 0;//_sensorsMap[SensorType::ANALOG][index]->update(sensorValue);
}

auto PhidgetIK::attachDelegate(CPhidgetHandle phid, void *userptr) -> int
{
	return ((PhidgetIK*) userptr)->attachHandler();
}

auto PhidgetIK::inputChangeDelegate(CPhidgetInterfaceKitHandle phid,
		void *userPtr, int index, int inputState) -> int
{
	return ((PhidgetIK*) userPtr)->inputChangeHandler(index, inputState);
}

auto PhidgetIK::outputChangeDelegate(CPhidgetInterfaceKitHandle phid,
		void *userPtr, int index, int outputState) -> int
{
	return ((PhidgetIK*) userPtr)->outputChangeHandler(index, outputState);
}

auto PhidgetIK::sensorChangeDelegate(CPhidgetInterfaceKitHandle phid,
		void *userPtr, int index, int sensorValue) -> int
{
	return ((PhidgetIK*) userPtr)->sensorChangeHandler(index, sensorValue);
}

auto PhidgetIK::addSensor(SensorType type, int index, std::string sensor_name) -> void
{
	Sensor* sensor;
	switch(type)
	{
		case SensorType::ANALOG:
			sensor = new AnalogInSensor(index, sensor_name, &_iKitHandle);
			break;
		case SensorType::DIGITAL_IN:
			sensor = new DigitalInSensor(index, sensor_name, &_iKitHandle);
			break;
		case SensorType::DIGITAL_OUT:
			sensor = new DigitalOutSensor(index, sensor_name, &_iKitHandle);
			break;
		default:
			sensor = nullptr;
			break;
	};
	//addSensor(sensor);
	if(sensor)
	{
		_sensorsMap.insert(std::make_pair(type, SensorMapInner()));
		_sensorsMap[type].insert(std::make_pair(index, sensor));
	}
}

auto PhidgetIK::addSensor(SensorType type, Sensor* sensor) -> void
{
	// unresoved overloaded function type here???
	// why?

	// if(sensor)
	// {
	// 	_sensorsMap.emplace(std::make_pair(type, SensorMapInner()));
	// 	_sensorsMap[type].emplace(std::make_pair(index, sensor));
	// }
}

auto PhidgetIK::removeSensor(SensorType type, int index) -> void
{
	_sensorsMap[type].erase(index);
}

auto PhidgetIK::update()-> void
{
	printf("PhidgetIK::update()");
}
