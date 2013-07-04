
#include "cob_sick_s300/SickS300.hpp"
namespace brics_oodl {

SickS300::SickS300() {
  // Bouml preserved body begin 00020E67

  this->sickS300 = NULL;
  this->config = NULL;
  this->isConnected = false;
  this->newDataFlagOne = false;
  this->newDataFlagTwo = false;
  timestampBufferOne = 0;
  timeNowBufferOne = 0;
  timestampBufferTwo = 0;
  timeNowBufferTwo = 0;

  //assigning zeros to the two buffers to reserve the memory
  distanceBufferOne.assign(numberOfScanPoints, 0);
  angleBufferOne.assign(numberOfScanPoints, 0);
  intensityBufferOne.assign(numberOfScanPoints, 0);

  distanceBufferTwo.assign(numberOfScanPoints, 0);
  angleBufferTwo.assign(numberOfScanPoints, 0);
  intensityBufferTwo.assign(numberOfScanPoints, 0);

  // Bouml preserved body end 00020E67
}

SickS300::~SickS300() {
  // Bouml preserved body begin 00020EE7
  Errors er;
  this->close(er);
  if (this->config != NULL) {
    delete this->config;
  }
  // Bouml preserved body end 00020EE7
}

bool SickS300::open(Errors& error) {
  // Bouml preserved body begin 00021367
  if (this->isConnected) {
    return true;
  }

  if (this->config->devicePath == "") {
    error.addError("no_DevicePath", "the device path is not specified in the configuration");
    this->isConnected = false;
    return false;
  }

  {
    boost::mutex::scoped_lock lock_it(mutexSickS300);

    if (sickS300 != NULL) {
      error.addError("still_Connected", "a previous connection was not closed correctly please close it again.");
      this->isConnected = false;
      return false;
    }

    this->sickS300 = new ScannerSickS300();

  }

  unsigned int desired_baud = 500000;

  switch (this->config->baud) {
    case BAUD_9600:
      desired_baud = 9600;
      LOG(trace) << "using 9600 baut to comunicate to Sick S300";
      break;
    case BAUD_19200:
      desired_baud = 19200;
      LOG(trace) << "using 19200 baut to comunicate to Sick S300";
      break;
    case BAUD_38400:
      desired_baud = 38400;
      LOG(trace) << "using 38400 baut to comunicate to Sick S300";
      break;
    case BAUD_115200:
      desired_baud = 115200;
      LOG(trace) << "using 115200 baut to comunicate to Sick S300";
      break;
    case BAUD_500K:
      desired_baud = 500000;
      LOG(trace) << "using 500000 baut to comunicate to Sick S300";
      break;
    case BAUD_UNKNOWN:
      desired_baud = 0;
      break;
  }

  //Initialize the Sick S300
  try {
    {
      boost::mutex::scoped_lock lock_it(mutexSickS300);
      if (!sickS300->open(this->config->devicePath.c_str(), desired_baud, this->config->scannerID)) {
        throw "could not initilize Sick S300";
      }
      this->isConnected = true;
    }
    LOG(trace) << "connection to Sick S300 initialized";

    stopThread = false;

    threads.create_thread(boost::bind(&SickS300::receiveScan, this));

  } catch (...) {
    error.addError("Initialize_failed", "could not initilize Sick S300");
    {
      boost::mutex::scoped_lock lock_it(mutexSickS300);
      this->isConnected = false;
      delete sickS300;
      sickS300 = NULL;
    }
    return false;
  }
  return true;
  // Bouml preserved body end 00021367
}

bool SickS300::close(Errors& error) {
  // Bouml preserved body begin 00020F67
  stopThread = true;

  threads.join_all();

  {
    boost::mutex::scoped_lock lock_it(mutexSickS300);
    if (sickS300 != NULL) {
      LOG(trace) << "connection to Sick S300 closed";
      delete sickS300;
      sickS300 = NULL;
    }
    this->isConnected = false;
  }
  return true;
  // Bouml preserved body end 00020F67
}

bool SickS300::setConfiguration(const LaserScannerConfiguration& configuration, Errors& error) {
  // Bouml preserved body begin 00020FE7

  if (this->config != NULL) {
    delete this->config;
  }
  this->config = new LaserScannerConfiguration;
  *(this->config) = configuration;

  if (!this->open(error)) {
    return false;
  }

  //  error.addError("configuration_not_possible", "the configuration is not possible.");
  return true;
  // Bouml preserved body end 00020FE7
}

bool SickS300::getConfiguration(LaserScannerConfiguration& configuration, Errors& error) {
  // Bouml preserved body begin 000210E7
  if (!this->open(error)) {
    return false;
  }
  try {
    configuration.vendor = "SICK";
    configuration.product = "S300";

    configuration.scanAngleStart = -135.0 / 180.0 * M_PI * radian;
    configuration.scanAngleStop = 135.0 / 180.0 * M_PI * radian;
    configuration.scanResolution = ((-configuration.scanAngleStart) + configuration.scanAngleStop) / (double) numberOfScanPoints;


    LOG(trace) << "read Sick LMS configuration";

  } catch (...) {
    error.addError("unable_to_read_configuration", "could not get the configuration from the Sick S300");
    return false;
  }

  return true;
  // Bouml preserved body end 000210E7
}

bool SickS300::getData(std::vector< double >& ranges_, std::vector< double >& rangeAngles_, std::vector< double >& intensities_, unsigned int& timestamp_, unsigned int& timeNow_, Errors& error) {

  if (!this->open(error)) {
    return false;
  }
  try {
    if (newDataFlagOne == true) {
      {
        boost::mutex::scoped_lock dataMutex1(mutexData1);
        ranges_ = distanceBufferOne;
        rangeAngles_ = angleBufferOne;
        intensities_ = intensityBufferOne;
        timestamp_ = timestampBufferOne;
        timeNow_ = timeNowBufferOne;
      }
      newDataFlagOne = false;

    } else if (newDataFlagTwo == true) {
      {
        boost::mutex::scoped_lock dataMutex2(mutexData2);
        ranges_ = distanceBufferTwo;
        rangeAngles_ = angleBufferTwo;
        intensities_ = intensityBufferTwo;
        timestamp_ = timestampBufferTwo;
        timeNow_ = timeNowBufferTwo;
      }
      newDataFlagTwo = false;
    } else {
     // error.addError("unable_to_get_data", "could not get data from the Sick S300");
      return false;
    }
    // LOG(trace) << "receiving range and intensity scan from Sick S300";

  } catch (...) {
    error.addError("unable_to_get_data", "could not get data from the Sick S300");
    return false;
  }

  return true;
}

bool SickS300::resetDevice(Errors& error) {
  // Bouml preserved body begin 000212E7
  error.addError("unable_to_reset_sick_s300", "could not reset the Sick S300");
  return false;
  // Bouml preserved body end 000212E7
}

void SickS300::receiveScan() {
  // Bouml preserved body begin 000371F1

  bool returnValue = false;

  {
    boost::mutex::scoped_lock lock_it(mutexSickS300);

    while (!stopThread) {

      if (newDataFlagOne == false) {
        {
          boost::mutex::scoped_lock dataMutex1(mutexData1);
          returnValue = sickS300->getScan(distanceBufferOne, angleBufferOne, intensityBufferOne, timestampBufferOne, timeNowBufferOne);
        }
        if (returnValue) {
          newDataFlagOne = true;
          newDataFlagTwo = false;
        }

      } else if (newDataFlagTwo == false) {
        {
          boost::mutex::scoped_lock dataMutex2(mutexData2);
          returnValue = sickS300->getScan(distanceBufferTwo, angleBufferTwo, intensityBufferTwo, timestampBufferTwo, timeNowBufferTwo);
        }
        if (returnValue) {
          newDataFlagTwo = true;
          newDataFlagOne = false;
        }
      }
      boost::this_thread::sleep(boost::posix_time::milliseconds(timeTillNextPollForData));
    }
  }
  // Bouml preserved body end 000371F1
}


} // namespace brics_oodl
