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

#include <cob_phidgets/phidget.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <cstring>

Phidget::Phidget(CPhidgetHandle* handle, SensingMode mode)
	: _phiHandle(handle), _serialNumber(-1), _last_error(-1), _sensMode(mode)
{
}

Phidget::~Phidget()
{
	CPhidget_close(*_phiHandle);
	CPhidget_delete(*_phiHandle);
}

auto Phidget::open(int serial_number) -> int
{
	return CPhidget_open(*_phiHandle, serial_number);
}

auto Phidget::close(int serial_number) -> int
{
	return CPhidget_close(*_phiHandle);
}

auto Phidget::waitForAttachment(int timeout) -> int
{
	return CPhidget_waitForAttachment(*_phiHandle, timeout);
}

auto Phidget::getDeviceType() -> std::string
{
	char a[256];
	const char * deviceptr = a;
	CPhidget_getDeviceType(*_phiHandle, &deviceptr);
	return std::string(deviceptr);
}

auto Phidget::getDeviceName() -> std::string
{
	char a[256];
	const char * deviceptr = a;
	CPhidget_getDeviceName(*_phiHandle, &deviceptr);
	return std::string(deviceptr);
}

auto Phidget::getDeviceLabel() -> std::string
{
	char a[256];
	const char * deviceptr = a;
	CPhidget_getDeviceType(*_phiHandle, &deviceptr);
	return std::string(deviceptr);
}

auto Phidget::getLibraryVersion() -> std::string
{
	char a[256];
	const char * deviceptr = a;
	CPhidget_getLibraryVersion(&deviceptr);
	return std::string(deviceptr);
}

auto Phidget::getDeviceSerialNumber() -> int
{
	int sernum;
	CPhidget_getSerialNumber(*_phiHandle, &sernum);
	_serialNumber = sernum;
	return sernum;
}

auto Phidget::getDeviceVersion() -> int
{
	int version;
	CPhidget_getDeviceVersion(*_phiHandle, &version);
	return version;
}

auto Phidget::getErrorDescription(int errorCode) -> std::string
{
	char a[256];
	const char * errorPtr = a;
	CPhidget_getErrorDescription(errorCode, &errorPtr);
	return std::string(errorPtr);
}

auto Phidget::attachHandler() -> int
{
	printf("attachHandler()");
	return 0;
}

auto Phidget::detachHandler() -> int
{
	printf("detachHandler()");
	return 0;
}

auto Phidget::update() -> void
{
	printf("Phidget::update()");
}
