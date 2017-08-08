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
 

#ifndef _PHIDGET_H_
#define _PHIDGET_H_

#include <string>
#include <libphidgets/phidget21.h>

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
