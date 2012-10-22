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

/*! \def USE_HAMMING
    the Hanning window will be used when filling the input array ( http://en.wikipedia.org/wiki/Window_function#Hann_window )
*/

#define USE_HANNING

/*! \def CLEAR_NOISE
    the FFT results can be misleading for the frequency band 0-30Hz) - this define will just set this to 0
*/
//#define CLEAR_NOISE

namespace mybeat
{

class FFT
{
public:

    FFT(uint16_t m_size);
    ~FFT();

    void setSoundBuffer(SoundBuffer *value){m_SoundBuffer=value;}

    void process_data();

    double get_element_r(uint16_t pos);

    double get_element_i(uint16_t pos);

    double get_magnitude(uint16_t pos);

    double get_magnitude_max();

private:

    SoundBuffer *m_SoundBuffer;

    uint16_t m_size;

    double *m_inputSignal;

    double *m_magnitude;

    double m_maxMagnitude;

    fftw_complex *m_outputSignal;

    fftw_plan plan_forward;
};
}

#endif
