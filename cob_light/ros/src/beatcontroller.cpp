
#include "beatcontroller.h"

namespace mybeat
{

BeatController::BeatController(uint16_t recordSize, uint32_t sampleRate, uint16_t m_bandCount)
{
    m_Buffer = new SoundBuffer(recordSize);
    m_Analyser = new BeatAnalyser(m_bandCount,sampleRate,recordSize);
#ifdef USE_ALSA
    dynamic_cast<AlsaRecorder*>(m_Recorder);
    m_Recorder = new AlsaRecorder(sampleRate,2,m_Buffer,recordSize);
#endif
#ifdef USE_PULSE
    dynamic_cast<PulseRecorder*>(m_Recorder);
    m_Recorder = new PulseRecorder(sampleRate,2,m_Buffer,recordSize);
#endif
    m_FFT = new FFT(recordSize);
    m_FFT->setSoundBuffer(m_Buffer);
    m_Analyser->setFFT(m_FFT);
    m_enabled=false;
}
BeatController::~BeatController()
{
    delete m_FFT;
    delete m_Recorder;
    delete m_Buffer;
    delete m_Analyser;
}
void BeatController::start()
{
    if(!m_enabled)
    {
        m_Recorder->start();
        m_enabled=true;
        //Connect SoundRecorder::newDataIsReady to processNewData()
        //connect(m_Recorder,SIGNAL(newDataIsReady()),this, SLOT(processNewData()));
        m_Recorder->signalNewDataIsReady()->connect(boost::bind(&BeatController::processNewData,this));
    }
}

void BeatController::stop()
{
    if(m_enabled)
    {
        m_Recorder->stop();
        m_enabled=false;
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
        m_FFT->process_data();
        m_Analyser->processData();
        m_sigProcessingDone();

        if(m_Analyser->getDrumBeat())
            m_sigBeatDrum();
        if(m_Analyser->getSnareBeat())
            m_sigBeatSnare();
        //Check for a beat for every frequency in our list
        std::tr1::unordered_set<uint16_t> myBeats;
        
        for(std::tr1::unordered_set<uint16_t>::iterator i = m_customBeats.begin();
            i != m_customBeats.end(); ++i)
        {
            if(m_Analyser->getBeatFrequency(*i))
                myBeats.insert(*i);
        }
        
        if(!myBeats.empty())
            m_sigCustom(myBeats);
    }
}
} //namespace libbeat
