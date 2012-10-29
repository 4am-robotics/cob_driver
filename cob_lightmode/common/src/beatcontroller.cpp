
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

#include "beatcontroller.h"

namespace mybeat
{

//                                          2048,               44100,              192
BeatController::BeatController(uint16_t recordSize, uint32_t sampleRate, uint16_t m_bandCount, uint16_t channels)
{
    for(int i = 0; i < channels; i++)
    {
        m_Buffers.push_back(new SoundBuffer(recordSize));
        m_FFTs.push_back(new FFT(recordSize));
        m_FFTs.back()->setSoundBuffer(m_Buffers.back());
        m_Analysers.push_back(new BeatAnalyser(m_bandCount,sampleRate,recordSize));
        m_Analysers.back()->setFFT(m_FFTs.back());
    }
    
#ifdef USE_ALSA
    dynamic_cast<AlsaRecorder*>(m_Recorder);
    m_Recorder = new AlsaRecorder(sampleRate, channels, m_Buffers, recordSize);
#endif
#ifdef USE_PULSE
    dynamic_cast<PulseRecorder*>(m_Recorder);
    m_Recorder = new PulseRecorder(sampleRate, channels, m_Buffers, recordSize);
#endif

    m_enabled=false;
}
BeatController::~BeatController()
{
    m_Recorder->stop();
    m_FFTs.clear();
    delete m_Recorder;
    m_Buffers.clear();
    m_Analysers.clear();
}
void BeatController::start()
{
    if(!m_enabled)
    {
        if(!m_Recorder->isStarted())
            m_Recorder->start();
        m_enabled=true;
        m_connectionProcessingDone = m_Recorder->signalNewDataIsReady()->connect(boost::bind(&BeatController::processNewData,this));
    }
}

void BeatController::stop()
{
    if(m_enabled)
    {
        m_enabled=false;
        m_connectionProcessingDone.disconnect();
    }
}
bool BeatController::getEnabled()
{
    return m_enabled;
}

void BeatController::processNewData()
{
    if(m_enabled)
    {
        for(int i=0; i<m_FFTs.size(); i++)
            m_FFTs.at(i)->process_data();

        for(int i=0; i<m_Analysers.size(); i++)
            m_Analysers.at(i)->processData();
        m_sigProcessingDone();

        if(m_Analysers.at(0)->getDrumBeat())
            m_sigBeatDrum();
        if(m_Analysers.at(0)->getSnareBeat())
            m_sigBeatSnare();
        //Check for a beat for every frequency in our list
        std::tr1::unordered_set<uint16_t> myBeats;
        
        for(std::tr1::unordered_set<uint16_t>::iterator i = m_customBeats.begin();
            i != m_customBeats.end(); ++i)
        {
            if(m_Analysers.at(0)->getBeatFrequency(*i))
                myBeats.insert(*i);
        }
        
        if(!myBeats.empty())
            m_sigCustom(myBeats);
    }
}
} //namespace libbeat
