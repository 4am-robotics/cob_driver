/*
 * test.cpp
 *
 *  Created on: 09-08-2010
 *  Author: Konrad Banachowicz
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

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
