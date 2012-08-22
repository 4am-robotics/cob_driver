#ifndef BEATANALYSER_H
#define BEATANALYSER_H
//#include <QVector>
#include <vector>
//#include <QDebug>
#include "subband.h"
#include "inttypes.h"
#include "fft.h"
#include "alsarecorder.h"

namespace mybeat
{

class BeatAnalyser
{
public:
    /*!
        @param m_bandCount in how many subbands the FFT result should be divided
        @param m_sampleRate
        @param m_recordSize how many samples you want to analyse
    */
    BeatAnalyser(uint16_t m_bandCount,uint32_t m_sampleRate, uint16_t m_recordSize);
    ~BeatAnalyser();
    /*!
        sets the FFT object address
        @param *value the pointer to the object
    */
    void setFFT(FFT *value){m_FFT=value;}
    /*!
        processes new data
    */
    void processData();
    /*!
        returns the number of bands
        @return number of bands
    */
    uint16_t getBands(){return m_bandCount;}
    /*!
        determine whether a beat has been detected in a band
        @param pos the band (0-(mbandCount-1))
        @return true if a beat has been detected, otherwise false
    */
    bool getBeat(uint16_t pos);
    /*!
        determine whether a beat has been detected in the corresponding band of a given frequency
        @param frequency
        @return true if a beat has been detected, otherwise false
    */
    bool getBeatFrequency(uint32_t frequency);
    /*!
        determine whether a beat has been detected for the drum (50Hz -> 200Hz)
        @return true if a beat has been detected, otherwise false
    */
    bool getDrumBeat();
    /*!
        determine whether a beat has been detected for the snare (1500Hz -> 2000Hz)
        @return true if a beat has been detected, otherwise false
    */
    bool getSnareBeat();
    /*!
        get raw access to a certain band
        @param pos the position of the band
        @return returns a pointer to the band, returns NULL if pos is out of range
    */
    SubBand* getBand(uint16_t pos);

private:
    uint16_t m_bandCount;
    uint32_t m_sampleRate;
    uint16_t m_recordSize;
    FFT *m_FFT;
    std::vector<SubBand*> *m_SubBands;
    //QVector<SubBand*> *m_SubBands;
    //QVector<bool> *m_beats;
    std::vector<bool> *m_beats;
};
}

#endif
