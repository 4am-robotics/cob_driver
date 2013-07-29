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