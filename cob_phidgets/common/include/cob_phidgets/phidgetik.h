#ifndef _PHIDGETIK_H_
#define _PHIDGETIK_H_

#include <cob_phidgets/phidget.h>

class PhidgetIK: public Phidget
{
public:
	PhidgetIK(SensingMode mode);
	~PhidgetIK();

	auto init(int serial_number) -> int;

	auto getInputCount() -> int;
	auto getOutputCount() -> int;
	auto getSensorCount() -> int;

	auto getInputState(int index) -> int;

	auto getOutputState(int index) -> int;
	auto setOutputState(int index, int state) -> int;

	auto getSensorValue(int index) -> int;
	auto getSensorRawValue(int index) -> int;

	auto getSensorChangeTrigger(int index) -> int;
	auto setSensorChangeTrigger(int index, int trigger) -> int;

	auto getRatiometric() -> int;
	auto setRatiometric(int ratiometric) -> int;

	auto getDataRate(int index) -> int;
	auto setDataRate(int index, int datarate) -> int;

	auto getDataRateMax(int index) -> int;
	auto getDataRateMin(int index) -> int;

	auto getError() -> int;

	auto addSensor(SensorType type, int index, std::string sensor_name) -> void;
	auto addSensor(SensorType type, Sensor* sensor) -> void;
	auto removeSensor(SensorType type, int index) -> void;

	virtual auto update() -> void;

protected:
	CPhidgetInterfaceKitHandle _iKitHandle;

	virtual int attachHandler();
	virtual int detachHandler();

	virtual int inputChangeHandler(int index, int inputState);
	virtual int outputChangeHandler(int index, int outputState);
	virtual int sensorChangeHandler(int index, int sensorValue);

private:
	static auto attachDelegate(CPhidgetHandle phid, void *userptr) -> int;

	static auto inputChangeDelegate(CPhidgetInterfaceKitHandle phid,
			void *userPtr, int index, int inputState) -> int;
	static auto outputChangeDelegate(CPhidgetInterfaceKitHandle phid,
			void *userPtr, int index, int outputState) -> int;
	static auto sensorChangeDelegate(CPhidgetInterfaceKitHandle phid,
			void *userPtr, int index, int sensorValue) -> int;
};
#endif //_PHIDGETIK_H_
