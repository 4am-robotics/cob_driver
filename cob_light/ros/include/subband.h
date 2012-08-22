#ifndef SUBBAND_H
#define SUBBAND_H
//Qt has nothing similar to <deque> so we simply use a <QList>
//#include <QList>
#include <deque>
#include <inttypes.h>

namespace mybeat
{

class SubBand
{
public:
    /*!
        @param m_size how many records SubBand needs to store (e.g. setting this to 4*samplerate/recordsize will result in 4 seconds of history)
        @param m_dropFactor (e.g. setting this to pow(0.5,(1/((double)samplerate/recordsize))) will result in 50% decrease after 2 seconds)
    */
    SubBand(uint16_t m_size=0,double m_dropFactor=0.985);
    /*!
        log a new record
        @param value the record to be logged - the oldest record will be removed
    */
    void log(double value);
    /*!
        returns the average magnitude for this SubBand
        @return the average magnitude
    */
    double average();
    /*!
        returns the highest magnitude so far. With each call the magnitude will be
        multiplied by m_dropFactor and therefore leading to a recalibration (gently reducing the magnitude)
        @return the highest magnitude so far
    */
    double getAllTimeMaximum();
    /*!
        return the highest magnitude so far without touching it. This can be used for displaying the current threshold.
        @return the highest magnitude so far
    */
    double getAllTimeMaximumRaw();
    /*!
        resets the maximum
    */
    void resetMaximum();

private:
    //QList<double> m_history;
    std::deque<double> m_history;
    uint16_t m_size;
    double m_allTimeMaximum;
    double m_dropFactor;
};
}
#endif
