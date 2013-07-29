#ifndef _PHIDGETMANAGER_H_
#define _PHIDGETMANAGER_H_

#include <phidget21.h>
#include <vector>
#include <string>

struct AttachedDevice
{
	int serial_num;
	std::string name;
};

class PhidgetManager
{
public:
	PhidgetManager();
	~PhidgetManager();

	auto getAttachedDevices()-> std::vector<AttachedDevice>;
private:
	CPhidgetManagerHandle _manHandle;
	std::vector<AttachedDevice> _attachedDevices;
};
#endif //_PHIDGETMANAGER_H_