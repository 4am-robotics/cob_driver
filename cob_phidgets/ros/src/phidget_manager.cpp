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

#include <ros/ros.h>
#include <cob_phidgets/phidget_manager.h>
#include <stdlib.h>
#include <unistd.h>

PhidgetManager::PhidgetManager()
	: _manHandle(0)
{
	CPhidgetManager_create(&_manHandle);
	CPhidgetManager_open((CPhidgetManagerHandle) _manHandle);

	sleep(2);
}

PhidgetManager::~PhidgetManager()
{
	// Close the manager    
	CPhidgetManager_close((CPhidgetManagerHandle) _manHandle);
	CPhidgetManager_delete((CPhidgetManagerHandle) _manHandle);

	sleep(0.5);
}

auto PhidgetManager::getAttachedDevices()-> std::vector<AttachedDevice>
{
	CPhidgetHandle* phidgetList;
	int count;
	ROS_INFO("getting attached Devices");
	CPhidgetManager_getAttachedDevices((CPhidgetManagerHandle) _manHandle, &phidgetList, &count);
 
 	std::vector<AttachedDevice> attachedDevices;
	int serialNumber;
	const char *name;

	// Iterate over the returned Phidget data
	for (int i = 0; i < count; i++) {
		CPhidget_getDeviceName(phidgetList[i], &name);
		CPhidget_getSerialNumber(phidgetList[i], &serialNumber);
		ROS_INFO("Found %s, with serial: %d", name, serialNumber);
		// Store name and serial number into a persistent variable
		AttachedDevice device{serialNumber, name};
		attachedDevices.push_back(device);
	}

	// Use the Phidget API to free the memory in the phidgetList Array    
	CPhidgetManager_freeAttachedDevicesArray(phidgetList);

	return attachedDevices;
}
