#include "beatanalyser.h"
namespace mybeat
{

BeatAnalyser::BeatAnalyser(uint16_t num_bands, uint32_t samplerate, uint16_t recordsize)
{
    this->m_bandCount=num_bands;
    this->m_sampleRate=samplerate;
    this->m_recordSize=recordsize;
    //m_SubBands = new QVector<SubBand*>;
    m_SubBands = new std::vector<SubBand*>;
    for(uint16_t i=0;i<num_bands;i++)
    {
        /*Create a new subband with 4 seconds of history and 50% decrease of the allTimeMaximum after 2 seconds*/
        SubBand *tmp = new SubBand(4*samplerate/recordsize, pow(0.5,(1/((double)samplerate/recordsize))));
        //m_SubBands->append(tmp);
        m_SubBands->push_back(tmp);
    }
    //m_beats = new QVector<bool> (num_bands,false);
    m_beats = new std::vector<bool> (num_bands,false);
}
BeatAnalyser::~BeatAnalyser()
{
    delete m_SubBands;
    delete m_beats;
}
void BeatAnalyser::processData()
{
    uint16_t freq_per_band=m_recordSize/2/m_bandCount;
    //m_beats->fill(false);
    m_beats->assign(m_beats->size(), false);
    for(uint16_t i=0;i<m_bandCount;i++)
    {
        double sum=0;
        for(uint16_t j=0;j<freq_per_band;j++)
        {
            sum+=m_FFT->get_magnitude(i*freq_per_band+j);
        }
        sum/=freq_per_band;
        m_SubBands->at(i)->log(sum);
        if(m_SubBands->at(i)->average() < sum && m_SubBands->at(i)->getAllTimeMaximum()*0.8 < sum)
        {
            //m_beats->replace(i,true);
            (*m_beats)[i] = true;
        }
    }
}
bool BeatAnalyser::getBeat(uint16_t pos)
{
    if(pos < m_bandCount)
        return m_beats->at(pos);
    else
        return false;
}
bool BeatAnalyser::getBeatFrequency(uint32_t frequency)
{
    uint16_t freq_per_band=m_sampleRate/m_bandCount/2;
    return m_beats->at((int)frequency/freq_per_band);
}
bool BeatAnalyser::getDrumBeat()
{
    /*We want to detect beats from 50Hz up to 200Hz*/
    bool tmp=false;
    uint16_t freq_per_band=m_sampleRate/m_bandCount/2;
    uint32_t low_limit=50/freq_per_band;
    uint32_t high_limit=200/freq_per_band;
    if(high_limit == 0)
       high_limit=1; /*If we have few bands, just take the first one*/

    for(uint16_t i=low_limit;i<high_limit;i++)
        tmp |= m_beats->at(i);
    return tmp;
}
bool BeatAnalyser::getSnareBeat()
{
    /*We want to detect beats from 1500Hz up to 2500Hz - this of course will return a beat more often due to the broad spectrum*/
    bool tmp=false;
    uint16_t freq_per_band=m_sampleRate/m_bandCount/2;
    uint32_t low_limit=1500/freq_per_band;
    uint32_t high_limit=2000/freq_per_band;
    for(uint16_t i=low_limit;i<high_limit;i++)
        tmp |= m_beats->at(i);
    return tmp;
}
SubBand* BeatAnalyser::getBand(uint16_t pos)
{
    if(pos < m_bandCount)
        return m_SubBands->at(pos);
    else
        return NULL;
}
} 
