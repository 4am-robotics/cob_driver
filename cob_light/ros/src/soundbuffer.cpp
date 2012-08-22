#include "soundbuffer.h"

namespace mybeat
{

SoundBuffer::SoundBuffer(uint16_t size)
{
    this->m_size=size;
    m_Buffer.resize(size);
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
