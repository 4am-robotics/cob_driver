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

	return state;
}

auto PhidgetIK::setOutputState(int index, int state) -> int
{
	return (_last_error = CPhidgetInterfaceKit_setOutputState(_iKitHandle,
	 		index, state));
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

	return value;
}

auto PhidgetIK::getSensorRawValue(int index) -> int
{
	int value = -1;

	 _last_error = CPhidgetInterfaceKit_getSensorRawValue(_iKitHandle, index, &value);

	return value;
}

auto PhidgetIK::getSensorChangeTrigger(int index) -> int
{
	int trigger = -1;

	 _last_error = CPhidgetInterfaceKit_getSensorChangeTrigger(_iKitHandle,	index, &trigger);

	return trigger;
}

auto PhidgetIK::setSensorChangeTrigger(int index, int trigger) -> int
{
	return (_last_error = CPhidgetInterfaceKit_setSensorChangeTrigger(_iKitHandle, index, trigger));
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

	return datarate;
}

auto PhidgetIK::setDataRate(int index, int datarate) -> int
{
	return (_last_error = CPhidgetInterfaceKit_setDataRate(_iKitHandle,	index, datarate));
}

auto PhidgetIK::getDataRateMax(int index) -> int
{
	int max = -1;

	_last_error = CPhidgetInterfaceKit_getDataRateMax(_iKitHandle, index, &max);

	return max;
}

auto PhidgetIK::getDataRateMin(int index) -> int
{
	int min = -1;

	_last_error = CPhidgetInterfaceKit_getDataRateMin(_iKitHandle, index, &min);

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
	return 0;
}

auto PhidgetIK::outputChangeHandler(int index, int outputState) -> int
{
	return 0;
}

auto PhidgetIK::sensorChangeHandler(int index, int sensorValue) -> int
{
	return 0;
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

auto PhidgetIK::update()-> void
{
	printf("PhidgetIK::update()");
}
