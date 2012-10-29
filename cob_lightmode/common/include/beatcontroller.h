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

#ifndef CONTROLLER_H
#define CONTROLLER_H
#define USE_ALSA

#ifdef USE_ALSA
#include "alsarecorder.h"
#endif
#ifdef USE_PULSE
#include "pulserecorder.h"
#endif
#include "fft.h"
#include "soundbuffer.h"
#include "beatanalyser.h"

#include <tr1/unordered_set>

namespace mybeat
{
    class BeatController
    {
    public:
        
        BeatController (uint16_t recordSize = 0, uint32_t sampleRate = 0, uint16_t m_bandCount = 0, uint16_t channels = 2);
        ~BeatController();

        std::vector<FFT*> getFFTs(){return m_FFTs;}

        std::vector<BeatAnalyser*> getAnalysers(){return m_Analysers;}

        std::vector<SoundBuffer*> getBuffers(){return m_Buffers;}

        void start();

        void stop();

        bool getEnabled();

        void addCustomBeat(uint16_t value){m_customBeats.insert(value);}

        void removeCustomBeat(uint16_t value){m_customBeats.erase(value);}

        boost::signals2::signal<void ()>* signalProcessingDone(){return &m_sigProcessingDone;}
        boost::signals2::signal<void ()>* signalBeatDrum(){return &m_sigBeatDrum;}
        boost::signals2::signal<void ()>* signalBeatSnare(){return &m_sigBeatSnare;}
        boost::signals2::signal<void (std::tr1::unordered_set<uint16_t>)>* signalBeatCustom(){return &m_sigCustom;}

    private:

        SoundRecorder *m_Recorder;
        std::vector<FFT*> m_FFTs;
        std::vector<SoundBuffer*> m_Buffers;
        std::vector<BeatAnalyser*> m_Analysers;
        bool m_enabled;

        std::tr1::unordered_set<uint16_t> m_customBeats;


        void processNewData();
        
        boost::signals2::connection m_connectionProcessingDone;
        boost::signals2::signal<void ()> m_sigProcessingDone;
        boost::signals2::signal<void ()> m_sigBeatDrum;
        boost::signals2::signal<void ()> m_sigBeatSnare;
        boost::signals2::signal<void (std::tr1::unordered_set<uint16_t>)> m_sigCustom;        
    };
}

#endif
