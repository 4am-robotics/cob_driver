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
 

#include <iostream>

#include <boost/lexical_cast.hpp>

#include <lms1xx.h>

void print_usage() {

	std::cout << " Usage : " << std::endl;
	std::cout << " set_config ip resolution rate " << std::endl;
	std::cout << " Exqample : " << std::endl;
	std::cout << " set_config 192.168.1.2 0.25 50 " << std::endl;
}

int main(int argc, char** argv) {

	LMS1xx laser;
	scanCfg sCfg;

	if(argc < 4) {
		print_usage();
		return 0;
	}

	laser.connect(argv[1]);

	if(!laser.isConnected()) {
		std::cout << "Unable to connect to device at address : " << argv[1] << std::endl;
		return 0;
	}

	sCfg.angleResolution = (int)(boost::lexical_cast<double>(std::string(argv[2])) * 10000);
	sCfg.scaningFrequency = boost::lexical_cast<int>(std::string(argv[3])) * 100;

	laser.login();
	laser.setScanCfg(sCfg);
	laser.saveConfig();

	sCfg = laser.getScanCfg();

	std::cout << "Configuration set to : " << std::endl;
	std::cout << "resolution : " << (double)sCfg.angleResolution/10000.0 << std::endl;
	std::cout << "frequency : " << (double)sCfg.scaningFrequency/100.0 << std::endl;

	laser.disconnect();

	return 0;
}
