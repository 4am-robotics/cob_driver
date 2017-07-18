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
 
 
#include "cob_sick_s300/LaserScannerConfiguration.hpp"
namespace brics_oodl {

LaserScannerConfiguration::LaserScannerConfiguration(){
  // Bouml preserved body begin 0001F47C
  // Bouml preserved body end 0001F47C
}

LaserScannerConfiguration::~LaserScannerConfiguration(){
  // Bouml preserved body begin 0001F4FC
  // Bouml preserved body end 0001F4FC
}

LaserScannerConfiguration::LaserScannerConfiguration(const LaserScannerConfiguration & source) {
  // Bouml preserved body begin 000214F1
  *this = source;
  // Bouml preserved body end 000214F1
}

LaserScannerConfiguration & LaserScannerConfiguration::operator=(const LaserScannerConfiguration & source) {
  // Bouml preserved body begin 00021571

  this->baud = source.baud;
  this->devicePath = source.devicePath;
  this->firmware = source.firmware;

  this->model = source.model;

  this->product = source.product;
  this->protocol = source.protocol;
  this->scanAngleStart = source.scanAngleStart;
  this->scanAngleStop = source.scanAngleStop;
  this->scanResolution = source.scanResolution;
  this->maxRangeDistance = source.maxRangeDistance;
  this->minRangeDistance = source.minRangeDistance;
  this->serialNumber = source.serialNumber;
  this->vendor = source.vendor;
  this->scannerID = source.scannerID;

  return *this;

  // Bouml preserved body end 00021571
}


} // namespace brics_oodl
