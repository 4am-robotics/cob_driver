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
 
 
// - Manager simple -
// This is a simple example showing how to setup a phidget manager and display a list of the currently connected
// Phidgets devices to the PC.


#include <ros/ros.h>

#include <stdio.h>
#include <cstdio>
#include <libphidgets/phidget21.h>

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
	const char* label;
	CPhidgetHandle *devices;

	CPhidgetManager_getAttachedDevices (MAN, &devices, &numDevices);

	printf("|-   # -|-        Label       -|-              Type              -|- Serial No. -|-  Version -|\n");
	printf("|-------|----------------------|----------------------------------|--------------|------------|\n");


	for(i = 0; i < numDevices; i++)
	{
		CPhidget_getDeviceType(devices[i], &ptr);
		CPhidget_getDeviceLabel(devices[i], &label);
		CPhidget_getSerialNumber(devices[i], &serialNo);
		CPhidget_getDeviceVersion(devices[i], &version);

		printf("|- %3d -|- %18s -|- %30s -|- %10d -|- %8d -|\n", i, label, ptr, serialNo, version);
		printf("|-------|----------------------|----------------------------------|--------------|------------|\n");
	}

	CPhidgetManager_freeAttachedDevicesArray(devices);

	printf("\nPress r to rename\n");
	printf("Press q to exit\n");

	return 0;
}

void set_label(CPhidgetManagerHandle MAN, int index)
{
	int numDevices;
	char choise;
	const char* label_old;
	std::string label_new;
	CPhidgetHandle *devices;

	printf("index: %d\n", index);

	CPhidgetManager_getAttachedDevices (MAN, &devices, &numDevices);
	CPhidget_getDeviceLabel(devices[index], &label_old);

	printf("\nenter new label: ");
	getline(std::cin, label_new);

	printf("\n old label: %s \n”", label_old);
	printf("new label: %s \n", label_new.c_str());
	printf("is this correct? [Y/n]: ");
	choise = getchar();
	getchar();
	switch(choise)
	{
		case '\n':
		case 'Y':
		case 'y':
			if(CPhidget_setDeviceLabel(devices[index], label_new.c_str()) == EPHIDGET_OK)
				printf("\nnew label is: %s \n", label_new.c_str());
			else
				printf("\nerror setting label!\n");
			break;
		case 'n':
			printf("\nlabel is still: %s \n", label_old);
			break;
		default:
			break;
	};

}

int set_device_label()
{
	int err;
	//Declare an Manager handle
	CPhidgetManagerHandle man = 0;

	CPhidget_enableLogging(PHIDGET_LOG_VERBOSE, NULL);

	//create the Manager object
	CPhidgetManager_create(&man);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	//CPhidgetManager_set_OnAttach_Handler(man, AttachHandler, man);
	//CPhidgetManager_set_OnDetach_Handler(man, DetachHandler, man);
	//CPhidgetManager_set_OnError_Handler(man, ErrorHandler, NULL);

	//open the Manager for device connections
	CPhidgetManager_open(man);

	sleep(10);

	if ((err = CPhidget_waitForAttachment((CPhidgetHandle) man, 10000))
			!= EPHIDGET_OK)
	{
		const char *errStr;
		CPhidget_getErrorDescription(err, &errStr);
		printf("Error waiting for attachment: (%d): %s", err, errStr);
	}

	char choise;
	display_devices(man);

	//end simulation
	choise = getchar();
	getchar();
	switch (choise)
	{
		case 'r':
			printf("Press index number of device you would like to rename\n");
			choise = getchar();
			getchar();
			set_label(man, atoi(&choise));
			break;
		case 'q':
			break;
		default:
			printf("Error\n");
	};
	//since user input has been read, this is a signal to terminate the program so we will close the phidget and delete the object we created
	printf("Closing...\n");
	CPhidgetManager_close(man);
	CPhidgetManager_delete(man);

	//all done, exit
	return 0;
}

int main(int argc, char* argv[])
{
	set_device_label();
	return 0;
}

