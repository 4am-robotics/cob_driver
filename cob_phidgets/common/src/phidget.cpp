/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_driver
 * \note
 *   ROS package name: cob_phidgets
 *
 * \author
 *   Author: Benjamin Maidel, mail: bnm@ipa.fhg.de
 * \author
 *   Supervised by:
 *
 * \date Date of creation: 29.07.2013
 *
 * \brief
 *   this package integrates phidget boards into the ros system
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

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
