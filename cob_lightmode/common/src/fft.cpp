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

    //This has to happen prior to any initialisation of the input array
    plan_forward = fftw_plan_dft_r2c_1d(m_size, m_inputSignal, m_outputSignal , 0);

}
FFT::~FFT()
{
    fftw_destroy_plan(plan_forward);
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
    
    //fill our array with data and apply windows if needed
#ifdef USE_BLACKMAN
    double a0 = (1-0.16)/2;
    double a1 = 0.5;
    double a2 = 0.16/2;
    for(uint16_t i=0; i<m_size; i++)
    {
        double mult = a0+a1*cos((2*M_PI*i)/(m_size-1))+a2*cos((4*M_PI*i)/(m_size-1));
        m_inputSignal[i] = (double)m_SoundBuffer->read(i) * mult;
    }
#endif
#ifdef USE_HANNING
    for(uint16_t i=0; i<m_size; i++)
    {
        double mult = 0.5 * (1.00 - cos(2*M_PI*i/m_size-1));
        m_inputSignal[i] = (double)m_SoundBuffer->read(i) * mult;
    }
#endif
#ifdef USE_NO_WINDOW
    for(uint16_t i=0; i<m_size;i++)
    {
        m_inputSignal[i]=(double)m_SoundBuffer->read(i);
    }
#endif

    fftw_execute(plan_forward);
    
    //Calculate Magnitude
    #ifdef CLEAR_NOISE
    //We delete some values since these will ruin our output
        m_outputSignal[0][0]=0;
        m_outputSignal[0][1]=0;
        m_outputSignal[1][0]=0;
        m_outputSignal[1][1]=0;
        m_outputSignal[2][0]=0;
        m_outputSignal[2][1]=0;
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
