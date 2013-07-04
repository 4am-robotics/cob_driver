/**
 * \file 
 *
 * \author
 * \date
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
