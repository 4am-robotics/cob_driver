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

#include "pulserecorder.h"
#include <stdio.h>
namespace mybeat
{


PulseRecorder::PulseRecorder(uint32_t samplerate,uint8_t channels, std::vector<SoundBuffer*> mySoundBuffers,uint16_t recordsize)
 :m_started(false)
{
    this->m_sampleRate=samplerate;
    this->m_SoundBuffers=mySoundBuffers;
    this->m_channels=channels;
    this->m_recordSize=recordsize;
    m_captureEnabled=false;
    m_signal = new int16_t[recordsize*channels];
}
PulseRecorder::~PulseRecorder()
{
    delete[] m_signal;
    m_captureEnabled=false;
    //Wait until run(); has finished
    wait();
}
void PulseRecorder::stop()
{
    m_captureEnabled=false;
}
bool PulseRecorder::initSound()
{
    m_ss.format= PA_SAMPLE_S16LE;
    m_ss.channels=m_channels;
    m_ss.rate=m_sampleRate;
    m_s = pa_simple_new(NULL, "mybeat",PA_STREAM_RECORD,NULL,"Sound Processing",&m_ss,NULL,NULL,NULL);
    if (m_s != NULL)
        return true; //No Error
    else
        return false; //Error
}
void PulseRecorder::closeSound()
{
    pa_simple_free(m_s);
}
void PulseRecorder::run()
{
    if(initSound())
    {
        m_started = true;
        m_captureEnabled=true;
        int error;
        while(m_captureEnabled)
        {
            if (pa_simple_read(m_s,m_signal,m_recordSize*m_channels*2,&error) < 0)
            {
                printf("pa_simple_read() failed: %s\n", pa_strerror(error));
                m_captureEnabled=false;
            }
            //Write data to Buffer
            for(uint16_t i=0;i<m_recordSize*m_channels;i+=m_channels)
            {
                int32_t sum=0;
                for(uint8_t j=0;j<m_channels;j++)
                    m_SoundBuffers.at(j)->write(i/m_channels,(int16_t)m_signal[i+j]);
            }
            //Emit signal and notify connected modules that new data is ready for processing
            m_sigNewDataReady();
        }
    }
    m_captureEnabled=false;
    closeSound();
}
} //namespace libbeat
