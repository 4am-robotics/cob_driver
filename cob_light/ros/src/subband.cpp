#include "subband.h"

namespace mybeat
{

SubBand::SubBand(uint16_t size,double dropFactor)
{
    this->m_size=size;
    this->m_dropFactor=dropFactor;
    m_allTimeMaximum=0;
}
void SubBand::log(double value)
{
    if(value > m_allTimeMaximum)
        m_allTimeMaximum=value;
     //Log our new value
     //m_history.prepend(value);
     m_history.push_front(value);
     //Delete the oldest record
     if(m_history.size() > m_size)
		m_history.pop_back();
         //m_history.removeLast();
     //else: We are still filling our list. this should only happend during the first second of recording
}
double SubBand::average()
{
     double sum=0;
     for(unsigned int i = 0; i < m_history.size(); i++)
     {
        sum+=m_history[i];
     }
     return sum/m_history.size();
}
double SubBand::getAllTimeMaximum()
{
    //With every call of this method we gradually lower the maximum to quickly adapt to changes in the input
    m_allTimeMaximum*=m_dropFactor;
    return m_allTimeMaximum;
}
double SubBand::getAllTimeMaximumRaw()
{
    //This function is for display purpose only. No recalibration will be performed
    return m_allTimeMaximum;
}
void SubBand::resetMaximum()
{
    m_allTimeMaximum=0;
}
}
