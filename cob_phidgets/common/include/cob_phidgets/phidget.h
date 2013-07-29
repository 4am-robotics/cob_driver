#ifndef _PHIDGET_H_
#define _PHIDGET_H_

#include <cob_phidgets/sensors.h>
#include <map>
#include <unordered_map>
#include <string>

class Phidget
{
public:
	virtual ~Phidget();

	auto open(int serial_number) -> int;
	auto close(int serial_number) -> int;
	auto waitForAttachment(int timeout) -> int;
	auto getDeviceType() -> std::string;
	auto getDeviceName() -> std::string;
	auto getDeviceLabel() -> std::string;
	auto getLibraryVersion() -> std::string;
	auto getDeviceSerialNumber() -> int;
	auto getDeviceVersion() -> int;

	static auto getErrorDescription(int errorCode) -> std::string;

protected:
	CPhidgetHandle* _phiHandle;
	int _serialNumber;
	int _last_error;

	struct SensorTypeHash
	{
		std::size_t operator()(const SensorType& type) const
		{
			return static_cast<size_t>(type);
		}
	};

	typedef std::unordered_map<int, Sensor*> SensorMapInner;
	typedef std::unordered_map<SensorType, SensorMapInner, SensorTypeHash> SensorMap;
	SensorMap _sensorsMap;
	

	Phidget(CPhidgetHandle * handle);

	virtual auto attachHandler() -> int;
	virtual auto detachHandler() -> int;

	virtual auto addSensor(SensorType type, int index, std::string sensor_name) -> void = 0;
	virtual auto addSensor(SensorType type, Sensor* sensor) -> void = 0;
	virtual auto removeSensor(SensorType type, int index) -> void = 0;
};
#endif //_PHIDGET_H_