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
