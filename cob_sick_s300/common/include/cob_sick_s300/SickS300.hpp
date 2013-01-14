#ifndef BRICS_OODL_SICKS300_H
#define BRICS_OODL_SICKS300_H


#include <vector>
#include <boost/thread.hpp>
#include "cob_sick_s300/Logger.hpp"
#include "cob_sick_s300/Units.hpp"
#include "cob_sick_s300/Errors.hpp"
//#include "cob_sick_s300/LaserScanner.hpp"
//#include "cob_sick_s300/LaserScannerData.hpp"
//#include "cob_sick_s300/LaserScannerDataWithIntensities.hpp"
#include "cob_sick_s300/LaserScannerConfiguration.hpp"
#include "cob_sick_s300/ScannerSickS300.h"


namespace brics_oodl {

/**
 * \brief 
 *
 */
class SickS300 {
  public:
    SickS300();

    virtual ~SickS300();

    bool open(Errors& error);

    bool close(Errors& error);

    bool setConfiguration(const LaserScannerConfiguration& configuration, Errors& error);

    bool getConfiguration(LaserScannerConfiguration& configuration, Errors& error);
    
    bool getData(std::vector< double >& ranges_, std::vector< double >& rangeAngles_, std::vector< double >& intensities_, unsigned int& timestamp_, unsigned int& timeNow_, Errors& error);

    bool resetDevice(Errors& error);


  private:
    void receiveScan();

    static const unsigned int numberOfScanPoints = 541;

    //in milliseconds
    static const unsigned int timeTillNextPollForData = 20;

    LaserScannerConfiguration* config;

    bool isConnected;

    ScannerSickS300* sickS300;

    std::vector<double> distanceBufferOne;

    std::vector<double> angleBufferOne;

    std::vector<double> intensityBufferOne;

    std::vector<double> distanceBufferTwo;

    std::vector<double> angleBufferTwo;

    std::vector<double> intensityBufferTwo;
    
    unsigned int timestampBufferOne;
	
    unsigned int timeNowBufferOne;
	
    unsigned int timestampBufferTwo;
	
    unsigned int timeNowBufferTwo;

    volatile bool stopThread;

    volatile bool newDataFlagOne;

    volatile bool newDataFlagTwo;

    boost::thread_group threads;

    boost::mutex mutexData1;

    boost::mutex mutexData2;

    boost::mutex mutexSickS300;

};

} // namespace brics_oodl
#endif
