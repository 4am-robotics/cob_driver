#include "fft.h"

namespace mybeat
{


FFT::FFT(uint16_t size=0)
{
    this->m_size=size;
    m_magnitude = new double[size/2];
    m_maxMagnitude=0;
    /*We should use fftw_malloc instead of new since fftw_malloc aligns the memory for optimal speed*/
    m_outputSignal = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * size);
    m_inputSignal = new double[size];
    m_SoundBuffer = NULL;

}
FFT::~FFT()
{
    delete [] m_inputSignal;
    delete [] m_magnitude;
    fftw_free(m_outputSignal);
}
void FFT::process_data()
{
    if(m_SoundBuffer == NULL)
    {
        printf("libbeat: No Soundbuffer has been set!");
        return;
    }
    //This has to happen prior to any initialisation of the input array
    fftw_plan plan_forward;
    plan_forward = fftw_plan_dft_r2c_1d(m_size, m_inputSignal, m_outputSignal , 0);
    //fill our array with data and apply windows if needed
#ifdef USE_BLACKMAN
    for(uint16_t i=0; i<m_size; i++)
    {
        double a0 = (1-0.16)/2;
        double a1 = 0.5;
        double a2 = 0.16/2;
        int16_t temp = m_SoundBuffer->read(i);
        m_inputSignal[i] = a0-a1*cos((2*M_PI*temp)/(m_size-1))+a2*cos((4*M_PI*temp)/(m_size-1));
    }
#endif
#ifdef USE_HANNING
    for(uint16_t i=0; i<m_size; i++)
    {
        m_inputSignal[i] = 0.5*(1.00-cos((2*M_PI*m_SoundBuffer->read(i))/(m_size-1)));
    }
#endif
#ifdef USE_NO_WINDOW
    for(uint16_t i=0; i<m_size;i++)
    {
        m_inputSignal[i]=(double)m_SoundBuffer->read(i);
    }
#endif

    fftw_execute(plan_forward);
    fftw_destroy_plan(plan_forward);
    //Calculate Magnitude
    #ifdef CLEAR_NOISE
    //We delete some values since these will ruin our output
        m_outputSignal[0][0]=0;
        m_outputSignal[0][1]=0;
        m_outputSignal[1][0]=0;
        m_outputSignal[1][1]=0;
    #endif
    for(uint16_t i=0;i<m_size/2;i++)
    {
        m_magnitude[i] = sqrt(pow(m_outputSignal[i][0],2)+pow(m_outputSignal[i][1],2));
        if(m_magnitude[i] > m_maxMagnitude)
            m_maxMagnitude=m_magnitude[i];
    }
}
double FFT::get_element_i(uint16_t pos)
{
    if(pos < m_size)
        return m_outputSignal[pos][1];
    else
        return 0;
}
double FFT::get_element_r(uint16_t pos)
{
    if(pos < m_size)
        return m_outputSignal[pos][0];
    else
        return 0;
}
double FFT::get_magnitude(uint16_t pos)
{
    if(pos < m_size/2)
        return m_magnitude[pos];
    else
        return 0;
}
double FFT::get_magnitude_max()
{
    return m_maxMagnitude;
}
} //namespace libbeat
