#ifndef ALSARECORDER_H
#define ALSARECORDER_H

#include <alsa/asoundlib.h>
#include <error.h>
#include <stdint.h>
#include "soundrecorder.h"
#include "soundbuffer.h"

namespace mybeat
{

class AlsaRecorder : public SoundRecorder
{
public:
    AlsaRecorder(uint32_t m_sampleSize,uint8_t m_channels,SoundBuffer *m_SoundBuffer,uint16_t m_recordSize);
    virtual ~AlsaRecorder();
    virtual void run();
    virtual void stop();
private:
    snd_pcm_t *m_captureHandle;
    uint16_t m_recordSize;
    uint32_t m_sampleSize;
    SoundBuffer *m_SoundBuffer;
    bool m_captureEnabled;
    uint8_t m_channels;
    int16_t *m_signal;
private:
    bool initSound();
    void closeSound();
};
}
#endif // ALSARECORDER_H
