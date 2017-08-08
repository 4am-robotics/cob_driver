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
 

#include "lms1xx.h"

#include <iostream>
#include <unistd.h>

int main()
{
	LMS1xx laser;
	scanData data;

	laser.connect("192.168.1.2");
	if(!laser.isConnected())
	{
		std::cout << "connection failend" << std::endl;
		return 0;
	}

	std::cout << "Connected to laser" << std::endl;

	std::cout << "Loging in ..." << std::endl;
	laser.login();

	laser.stopMeas();

	std::cout << "Geting scan configuration ..." << ::std::endl;
	scanCfg c = laser.getScanCfg();

	//std::cout << "Scanning Frequency : " << c.scaningFrequency/100.0 << "Hz AngleResolution : " << c.angleResolution/10000.0 << "deg " << std::endl;

	c.angleResolution = 5000;
	c.scaningFrequency = 5000;

	laser.setScanCfg(c);

	scanDataCfg cc;
	cc.deviceName = false;
	cc.encoder = 0;
	cc.outputChannel = 3;
	cc.remission = true;
	cc.resolution = 0;
	cc.position = false;
	cc.outputInterval = 1;

	laser.setScanDataCfg(cc);

	int ret = 0;
	std::cout << "Start measurements ..." << std::endl;
	laser.startMeas();

	std::cout << "Wait for ready status ..." << std::endl;
	ret = 0;
	while (ret != 7)
	{
		ret = laser.queryStatus();
		std::cout << "status : " << ret << std::endl;
		sleep(1);
	}
	std::cout << "Laser ready" << std::endl;

	std::cout << "Start continuous data transmission ..." << std::endl;
	laser.scanContinous(1);

	for(int i =0; i < 3; i++)
	{
		std::cout << "Receive data sample ..." << std::endl;
		laser.getData(data);
	}

	std::cout << "Stop continuous data transmission ..." << std::endl;
	laser.scanContinous(0);

	laser.stopMeas();

	std::cout << "Disconnect from laser" << std::endl;
	laser.disconnect();

	return 0;
}
