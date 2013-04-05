/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering   
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_light
 * Description: Switch robots led color by sending data to
 * the led-µC over serial connection.
 *                              
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *          
 * Author: Benjamin Maidel, email:benjamin.maidel@ipa.fraunhofer.de
 * Supervised by: Benjamin Maidel, email:benjamin.maidel@ipa.fraunhofer.de
 *
 * Date of creation: August 2012
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
 
#include "beatanalyser.h"
namespace mybeat
{

BeatAnalyser::BeatAnalyser(uint16_t num_bands, uint32_t samplerate, uint16_t recordsize)
: m_maxBandValue(0.0)
{
    this->m_bandCount=num_bands;
    this->m_sampleRate=samplerate;
    this->m_recordSize=recordsize;
    m_SubBands = new std::vector<Band*>;
    for(uint16_t i=0;i<num_bands;i++)
    {
        /*Create a new subband with 4 seconds of history and 50% decrease of the allTimeMaximum after 2 seconds*/
        Band *tmp = new Band(4*samplerate/recordsize, pow(0.5,(1/((double)samplerate/recordsize))));
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
    uint16_t freq_per_band=m_recordSize/2/m_bandCount/8;
    //m_beats->fill(false);
    m_beats->assign(m_bandCount, false);
    for(uint16_t i=0;i<m_bandCount;i++)
    {
        double sum=0;
        for(uint16_t j=0;j<freq_per_band;j++)
        {
            sum+=m_FFT->get_magnitude(i*freq_per_band+j);
        }
        sum/=freq_per_band;
        if(sum > m_maxBandValue)
            m_maxBandValue = sum;

        m_SubBands->at(i)->log(sum);
        if(m_SubBands->at(i)->average() < sum && m_SubBands->at(i)->getAllTimeMaximum()*0.8 < sum)
        {
            //m_beats->replace(i,true);
            (*m_beats)[i] = true;
        }
    }
}
double BeatAnalyser::getMagSpectrum(uint32_t low, uint32_t high)
{
    static double magSpecMax = 0;
    double magSpec=0;
    uint16_t freq_per_band= m_sampleRate/m_bandCount;
    uint32_t low_limit=low/freq_per_band;
    uint32_t high_limit=high/freq_per_band;
    double avgAllTimeMax = 0;
    double avgAverage = 0;
    
    for(uint16_t i=low_limit; i<high_limit;i++)
    {
        avgAllTimeMax+= m_SubBands->at(i)->getAllTimeMaximumRaw();
        if(m_SubBands->at(i)->getNewest() > avgAverage)
            avgAverage = m_SubBands->at(i)->getNewest();
    }
    magSpec = avgAverage / (avgAllTimeMax/(high_limit-low_limit));
    if(magSpec > magSpecMax)
        magSpecMax = magSpec;
    else
        magSpecMax *= 0.8;
    magSpec = magSpec / magSpecMax;
    //printf("magSpec: %f\n", magSpec);
    return magSpec;
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
    uint16_t freq_per_band=m_sampleRate/m_bandCount;
    return m_beats->at((int)frequency/freq_per_band);
}
bool BeatAnalyser::getDrumBeat()
{
    /*We want to detect beats from 50Hz up to 200Hz*/
    bool tmp=false;
    uint16_t freq_per_band=m_sampleRate/m_bandCount;
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
    uint16_t freq_per_band=m_sampleRate/m_bandCount;
    uint32_t low_limit=1500/freq_per_band;
    uint32_t high_limit=2000/freq_per_band;
    for(uint16_t i=low_limit;i<high_limit;i++)
        tmp |= m_beats->at(i);
    return tmp;
}
Band* BeatAnalyser::getBand(uint16_t pos)
{
    if(pos < m_bandCount)
        return m_SubBands->at(pos);
    else
        return NULL;
}
} 
