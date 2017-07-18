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
 

#ifndef _PHIDGETMANAGER_H_
#define _PHIDGETMANAGER_H_

#include <vector>
#include <string>
#include <cob_phidgets/phidget.h>

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
