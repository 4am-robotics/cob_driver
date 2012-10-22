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

#include "band.h"

namespace mybeat
{

Band::Band(size_t size,double dropFactor)
 : m_size(size), m_dropFactor(dropFactor), m_allTimeMaximum(0.0)
{
}

void Band::log(double value)
{
    if(value > m_allTimeMaximum)
        m_allTimeMaximum=value;

     m_history.push_front(value);

     if(m_history.size() > m_size)
		m_history.pop_back();
}

double Band::average()
{
     double sum=0;
     for(unsigned int i = 0; i < m_history.size(); i++)
     {
        sum+=m_history[i];
     }
     return sum/m_history.size();
}

double Band::getAllTimeMaximum()
{
    //With every call of this method we gradually lower the maximum to quickly adapt to changes in the input
    m_allTimeMaximum*=m_dropFactor;
    return m_allTimeMaximum;
}

double Band::getAllTimeMaximumRaw()
{
    //This function is for display purpose only. No recalibration will be performed
    return m_allTimeMaximum;
}

void Band::resetMaximum()
{
    m_allTimeMaximum=0;
}

std::deque<double> Band::getHistory()
{
    return m_history;
}

double Band::getNewest()
{
    return m_history.front();
}

double Band::getOldest()
{
    return m_history.back();
}
}
