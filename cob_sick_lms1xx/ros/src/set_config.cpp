/*
 * setConfig.cpp
 *
 *  Created on: 09-09-2011
 *      Author: konradb3
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
