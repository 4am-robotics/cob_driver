#ifndef _PHIDGET_H_
#define _PHIDGET_H_

#include <string>
#include <phidget21.h>

class Phidget
{
public:
	enum class SensingMode{EVENT=0, POLLING=1};

	~Phidget();

	auto open(int serial_number) -> int;
	auto close(int serial_number) -> int;
	auto waitForAttachment(int timeout) -> int;
	auto getDeviceType() -> std::string;
	auto getDeviceName() -> std::string;
	auto getDeviceLabel() -> std::string;
	auto getLibraryVersion() -> std::string;
	auto getDeviceSerialNumber() -> int;
	auto getDeviceVersion() -> int;

	virtual auto update() -> void;

	static auto getErrorDescription(int errorCode) -> std::string;

protected:
	CPhidgetHandle* _phiHandle;
	int _serialNumber;
	int _last_error;
	SensingMode _sensMode;

	Phidget(CPhidgetHandle * handle, SensingMode mode);

	virtual auto attachHandler() -> int;
	virtual auto detachHandler() -> int;
};
#endif //_PHIDGET_H_
