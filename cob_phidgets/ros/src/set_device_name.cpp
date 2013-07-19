// - Manager simple -
// This is a simple example showing how to setup a phidget manager and display a list of the currently connected
// Phidgets devices to the PC.
//
// Copyright 2008 Phidgets Inc.  All rights reserved.
// This work is licensed under the Creative Commons Attribution 2.5 Canada License. 
// view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/

#include <ros/ros.h>

#include <stdio.h>
#include <phidget21.h>

int display_devices(CPhidgetManagerHandle MAN);

int AttachHandler(CPhidgetHandle phid, void *userPtr)
{
	int serialNo;
	const char *name;
	CPhidget_DeviceID id;
	CPhidget_DeviceClass cls;

	CPhidget_getDeviceName (phid, &name);
	CPhidget_getSerialNumber(phid, &serialNo);
	CPhidget_getDeviceClass(phid, &cls);
	CPhidget_getDeviceID(phid, &id);

	printf("%s %10d attached! (%d, %d) \n", name, serialNo, cls, id);

	display_devices((CPhidgetManagerHandle)userPtr);
	return 0;
}

int DetachHandler(CPhidgetHandle phid, void *userPtr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (phid, &name);
	CPhidget_getSerialNumber(phid, &serialNo);
	printf("%s %10d detached!\n", name, serialNo);

	display_devices((CPhidgetManagerHandle)userPtr);
	return 0;
}

int ErrorHandler(CPhidgetManagerHandle MAN, void *usrptr, int Code, const char *Description)
{
	printf("Error handled. %d - %s\n", Code, Description);
	return 0;
}

//Display the properties of the attached phidget(s) to the screen.  We will be displaying the name, serial number and version of the attached device(s).
int display_devices(CPhidgetManagerHandle MAN)
{
	int serialNo, version, numDevices, i;
	const char* ptr;
	CPhidgetHandle *devices;

	CPhidgetManager_getAttachedDevices (MAN, &devices, &numDevices);

	printf("|-   # -|-              Type              -|- Serial No. -|-  Version -|\n");
	printf("|-------|----------------------------------|--------------|------------|\n");


	for(i = 0; i < numDevices; i++)
	{
		CPhidget_getDeviceType(devices[i], &ptr);
		CPhidget_getSerialNumber(devices[i], &serialNo);
		CPhidget_getDeviceVersion(devices[i], &version);

		printf("|- %3d -|- %30s -|- %10d -|- %8d -|\n", i, ptr, serialNo, version);
		printf("|-------|----------------------------------|--------------|------------|\n");
	}

	CPhidgetManager_freeAttachedDevicesArray(devices);

	return 0;
}

int set_device_names()
{
	//Declare an Manager handle
	CPhidgetManagerHandle man = 0;

	CPhidget_enableLogging(PHIDGET_LOG_VERBOSE, NULL);

	//create the Manager object
	CPhidgetManager_create(&man);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidgetManager_set_OnAttach_Handler(man, AttachHandler, man);
	CPhidgetManager_set_OnDetach_Handler(man, DetachHandler, man);
	CPhidgetManager_set_OnError_Handler(man, ErrorHandler, NULL);

	//open the Manager for device connections
	CPhidgetManager_open(man);
	//end simulation
	printf("Press any key to end\n");
	getchar();

	//since user input has been read, this is a signal to terminate the program so we will close the phidget and delete the object we created
	printf("Closing...\n");
	CPhidgetManager_close(man);
	CPhidgetManager_delete(man);

	//all done, exit
	return 0;
}

int main(int argc, char* argv[])
{
	set_device_names();
	return 0;
}

