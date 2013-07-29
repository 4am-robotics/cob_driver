#ifndef _PHIDGETIK_H_
#define _PHIDGETIK_H_

#include <cob_phidgets/phidget.h>

class PhidgetIK: public Phidget
{
public:

	PhidgetIK();

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

	auto addSensor(SensorType type, int index, std::string sensor_name) override -> void;
	auto addSensor(SensorType type, Sensor* sensor) override -> void;
	auto removeSensor(SensorType type, int index) override -> void;

protected:
	CPhidgetInterfaceKitHandle _iKitHandle;

	virtual auto attachHandler() override -> int;
	virtual auto detachHandler() override -> int;

	virtual auto inputChangeHandler(int index, int inputState) -> int;
	virtual auto outputChangeHandler(int index, int outputState) -> int;
	virtual auto sensorChangeHandler(int index, int sensorValue) -> int;

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