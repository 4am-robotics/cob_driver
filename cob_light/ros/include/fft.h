#ifndef FFT_H
#define FFT_H
#include <fftw3.h>
#include <inttypes.h>
#include <cmath>
#include "soundbuffer.h"
/*! \def USE_NO_WINDOW
    no window ( http://en.wikipedia.org/wiki/Window_function ) will be used when filling the input array
*/

/*! \def USE_BLACKMAN
    the Blackman window will be used when filling the input array ( http://en.wikipedia.org/wiki/Window_function#Blackman_windows )
*/

/*! \def USE_HANNING
    the Hanning window will be used when filling the input array ( http://en.wikipedia.org/wiki/Window_function#Hann_window )
*/

#define USE_HANNING

/*! \def CLEAR_NOISE
    the FFT results can be misleading for the frequency band 0-30Hz) - this define will just set this to 0
*/
#define CLEAR_NOISE

namespace mybeat
{

class FFT
{
public:
    /*!
        creates a FFT of a certain size
        @param m_size
    */
    FFT(uint16_t m_size);
    ~FFT();
    /*!
        sets the address of m_SoundBuffer
        @param *value
    */
    void setSoundBuffer(SoundBuffer *value){m_SoundBuffer=value;}
    /*!
        starts processing of new data if m_SoundBuffer is not NULL
    */
    void process_data();
    /*!
        provides access to the real part of the processed signal
        @param pos the position in the FFT, valid input is 0-(m_size-1)
        @return the value at the position. if the position is out of range, return will be 0
    */
    double get_element_r(uint16_t pos);
    /*!
        provides access to the imaginary part of the processed signal
        @param pos the processed signal at pos, valid input is 0-(m_size-1)
        @return the value at pos. if pos is out of range, return will be 0
    */
    double get_element_i(uint16_t pos);
    /*!
        provides access to the magnitude of the signal
        @param pos the processed signal at pos, valid input is 0-(m_size/2)
        @return the value at the pos. if pos is out of range, return will be 0
    */
    double get_magnitude(uint16_t pos);
    /*!
        provides the highest magnitude of the whole signal. This can be used for scaling graphs
        @return highest magnitude
    */
    double get_magnitude_max();

private:
    /*! the SoundBuffer from where FFT gets new data */
    SoundBuffer *m_SoundBuffer;
    /*! the size of the input signal */
    uint16_t m_size;
    /*! the input signal */
    double *m_inputSignal;
    /*! an array where the magnitude will be stored */
    double *m_magnitude;
    /*! the maximum magnitude inside m_magnitude */
    double m_maxMagnitude;
    /*! the results of fftw */
    fftw_complex *m_outputSignal;
};
}

#endif
