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

#include "soundbuffer.h"
#include "stdio.h"

namespace mybeat
{

SoundBuffer::SoundBuffer(uint16_t size)
: m_maxPwr (0.0), m_pwr(0), m_averagePwr(0), m_average(0), m_newDataReady(false)
{
    this->m_size=size;
    m_Buffer.resize(size,0);
}

int16_t SoundBuffer::average()
{
    int32_t sum=0;
    for(uint16_t i=0;i<m_size;i++)
    {
        sum+=m_Buffer[i];
    }
    sum/=m_size;
    return (int16_t)sum;
}

double SoundBuffer::pwr()
{
    double pwr=0;
    for(uint16_t i=0; i<m_size; i++)
    {
        pwr+=(double)m_Buffer[i] * (double)m_Buffer[i];
    }
    if(pwr > m_maxPwr)
        m_maxPwr = pwr;
    return pwr;
}

double SoundBuffer::max_pwr()
{
    m_maxPwr = m_maxPwr*0.95;
    return m_maxPwr;
}

uint16_t SoundBuffer::average_pwr()
{
    uint32_t sum=0;
    for(uint16_t i=0;i<m_size;i++)
    {
        if(m_Buffer[i] < 0)
            sum+=-1*m_Buffer[i];
        else
            sum+=m_Buffer[i];
    }
    sum/=m_size;
    return (uint16_t)sum;
}
bool SoundBuffer::write(uint16_t pos,int16_t value)
{
        if(pos < m_size)
        {
            m_Buffer[pos]=value;
            return true;
        }
        else
            return false;
}
int16_t SoundBuffer::read(uint16_t pos)
{
        return m_Buffer.at(pos);
}
}
