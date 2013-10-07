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
