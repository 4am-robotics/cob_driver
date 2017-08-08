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
 

#ifndef BRICS_OODL_LASERSCANNERCONFIGURATION_H
#define BRICS_OODL_LASERSCANNERCONFIGURATION_H

/**
 * \file
 *
 * \author
 * \date
 */
#include <vector>
#include <iostream>
#include <string>
#include "cob_sick_s300/Units.hpp"
namespace brics_oodl {

enum baud_rate {
  BAUD_9600,
  BAUD_19200,
  BAUD_38400,
  BAUD_115200,
  BAUD_500K,
  BAUD_UNKNOWN

};
/**
 * \brief
 *
 */
class LaserScannerConfiguration {
  public:
    LaserScannerConfiguration();

    virtual ~LaserScannerConfiguration();

    LaserScannerConfiguration(const LaserScannerConfiguration & source);

    virtual LaserScannerConfiguration & operator=(const LaserScannerConfiguration & source);

    /// Vendor name.
    std::string vendor;

    /// Product name.
    std::string product;

    /// Firmware version.
    std::string firmware;

    std::string protocol;

    std::string serialNumber;

    std::string model;

    quantity<plane_angle> scanAngleStart;

    quantity<plane_angle> scanAngleStop;

    quantity<plane_angle> scanResolution;

    quantity<length> minRangeDistance;

    quantity<length> maxRangeDistance;

    baud_rate baud;

    std::string devicePath;

    int scannerID;

};

} // namespace brics_oodl
#endif
