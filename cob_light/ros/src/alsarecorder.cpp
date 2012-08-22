#include <alsarecorder.h>

namespace mybeat
{

AlsaRecorder::AlsaRecorder(uint32_t samplerate, uint8_t channels, SoundBuffer *mySoundBuffer,uint16_t recordsize)
{
    this->m_sampleSize=samplerate;
    this->m_SoundBuffer=mySoundBuffer;
    this->m_channels=channels;
    this->m_recordSize=recordsize;
    m_captureEnabled=false;
    m_signal = new int16_t[recordsize*channels];
}

AlsaRecorder::~AlsaRecorder()
{
    delete[] m_signal;
    m_captureEnabled=false;
    //Wait until run(); has finished
    this->join();
}
void AlsaRecorder::stop()
{
    m_captureEnabled=false;
}
bool AlsaRecorder::initSound()
{
    snd_pcm_hw_params_t *hw_params;
    char pcm_name[]="default";
    int err;


    if ((err = snd_pcm_open (&m_captureHandle, pcm_name, SND_PCM_STREAM_CAPTURE, 0)) < 0)
    {
        printf("cannot open audio device (%s)\n",
                 snd_strerror (err));
        return false;
    }
    if ((err = snd_pcm_hw_params_malloc (&hw_params)) < 0)
    {
        printf("cannot allocate hardware parameter structure (%s)\n",
                 snd_strerror (err));
        return false;
    }
    if ((err = snd_pcm_hw_params_any (m_captureHandle, hw_params)) < 0)
    {
        printf("cannot initialize hardware parameter structure (%s)\n",
                 snd_strerror (err));
        return false;
    }
    if ((err = snd_pcm_hw_params_set_access (m_captureHandle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0)
    {
        printf("cannot set access type (%s)\n",
                 snd_strerror (err));
        return false;
    }
    if ((err = snd_pcm_hw_params_set_format (m_captureHandle, hw_params, SND_PCM_FORMAT_S16_LE)) < 0)
    {
        printf("cannot set sample format (%s)\n",
                 snd_strerror (err));
        return false;
    }
    if ((err = snd_pcm_hw_params_set_rate_near (m_captureHandle, hw_params, (unsigned int *) &m_sampleSize, 0)) < 0)
    {
        printf("cannot set sample rate (%s)\n",
                 snd_strerror (err));
        return false;
    }
    if ((err = snd_pcm_hw_params_set_channels (m_captureHandle, hw_params, m_channels)) < 0)
    {
        printf("cannot set channel count (%s)\n",
                 snd_strerror (err));
        return false;
    }
    if ((err = snd_pcm_hw_params (m_captureHandle, hw_params)) < 0)
    {
        printf("cannot set parameters (%s)\n",
                 snd_strerror (err));
        return false;
    }
    snd_pcm_hw_params_free (hw_params);
    return true;
}
void AlsaRecorder::closeSound()
{
    snd_pcm_close (m_captureHandle);
}
void AlsaRecorder::run()
{
    int err;
    //stop will set this to false
    m_captureEnabled=true;
    if(initSound())
    {
        while(m_captureEnabled)
        {
            if ((err = snd_pcm_readi (m_captureHandle, m_signal, m_recordSize)) != m_recordSize)
            {
                if ((err = snd_pcm_prepare (m_captureHandle)) < 0)
                {
                    printf("cannot prepare audio interface for use (%s)\n",
                             snd_strerror (err));

                }
            }
            //Write data to Buffer
            for(uint16_t i=0;i<m_recordSize*m_channels;i+=m_channels)
            {
                int32_t sum=0;
                for(uint8_t j=0;j<m_channels;j++)
                    sum+=m_signal[i+j];
                m_SoundBuffer->write(i/m_channels,(int16_t)sum/m_channels);
            }
            //Emit signal and notify connected modules that new data is ready for processing
            m_sigNewDataReady();
        }
    }
    closeSound();
}
} //namespace mybeat
