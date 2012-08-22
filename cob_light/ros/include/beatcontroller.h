/*
    This file is part of libbeat - a lightweight beat detection library
    Copyright (c) 2011 by Maximilian Güntner <maximilian.guentner@gmail.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
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

//#include <QTimerEvent>
//#include <QSet>
#include <tr1/unordered_set>

namespace mybeat
{

class BeatController
{
public:
    /*!
        creates a controller for all other components of libbeat.
        @param m_bandCount in how many subbands the input signal should be divided
        @param recordSize how many samples you want to record and analyse at one go (4096 is a good value for 44100 Hz sampleRate)
        @param sampleRate
        @param *parent
    */
    explicit BeatController (uint16_t recordSize = 0, uint32_t sampleRate = 0, uint16_t m_bandCount = 0);
    ~BeatController();
    /*!
        provides raw access to m_FFT
        @returns a pointer to m_FFT
    */
    FFT* getFFT(){return m_FFT;}
    /*!
        provides raw access to m_Analyser
        @returns a pointer to m_Analyser
    */
    BeatAnalyser* getAnalyser(){return m_Analyser;}
    /*!
        provides raw access to m_Buffer
        @returns a pointer to m_Buffer
    */
    SoundBuffer* getBuffer(){return m_Buffer;}
    /*!
        starts recording and processing
    */
    void start();
    /*!
        stops recording and processing
    */
    void stop();
    /*!
        get the current state of the controller
        @returns true if the controller is active, false if not
    */
    bool getEnabled();
    /*!
        adds a custom frequency for which a signal will be set up
        @param value the target frequency to be added
    */
    void addCustomBeat(uint16_t value){m_customBeats.insert(value);}
    /*!
        removes a custom frequency
        @param value the target frequency to be removed
    */
    void removeCustomBeat(uint16_t value){m_customBeats.erase(value);}
private:
    /*!
        the SoundRecorder connected to this BeatController
    */
    SoundRecorder *m_Recorder;
    FFT *m_FFT;
    SoundBuffer *m_Buffer;
    BeatAnalyser *m_Analyser;
    bool m_enabled;
    /*! this QSet contains all custom beats. */
    //QSet<uint16_t> m_customBeats;
    std::tr1::unordered_set<uint16_t> m_customBeats;


    void processNewData();
    
    boost::signals2::signal<void ()> m_sigProcessingDone;
    boost::signals2::signal<void ()> m_sigBeatDrum;
    boost::signals2::signal<void ()> m_sigBeatSnare;
    boost::signals2::signal<void (std::tr1::unordered_set<uint16_t>)> m_sigCustom;

public:

    boost::signals2::signal<void ()>* signalProcessingDone(){return &m_sigProcessingDone;}
    boost::signals2::signal<void ()>* signalBeatDrum(){return &m_sigBeatDrum;}
    boost::signals2::signal<void ()>* signalBeatSnare(){return &m_sigBeatSnare;}
    boost::signals2::signal<void (std::tr1::unordered_set<uint16_t>)>* signalBeatCustom(){return &m_sigCustom;}
    
};
}

#endif // CONTROLLER_H
