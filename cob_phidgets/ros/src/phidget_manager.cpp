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

	usleep(500000);	//0.5s
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
