#ifndef SOUNDRECORDER_H
#define SOUNDRECORDER_H
#include <inttypes.h>
#include <threadbase.h>
#include <boost/signals2.hpp>

namespace mybeat
{
class SoundRecorder : public ThreadBase
{
public:
    SoundRecorder();
    virtual ~SoundRecorder();
    virtual void stop() = 0;

	boost::signals2::signal<void ()>* signalNewDataIsReady(){return &m_sigNewDataReady;}

protected:
	
	boost::signals2::signal<void ()> m_sigNewDataReady;

};
}
#endif // SOUNDRECORDER_H
