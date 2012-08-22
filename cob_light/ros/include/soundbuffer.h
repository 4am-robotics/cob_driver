#ifndef SOUNDBUFFER_H
#define SOUNDBUFFER_H
#include <inttypes.h>
#include <vector>

namespace mybeat
{

class SoundBuffer
{

public:

    SoundBuffer(uint16_t m_size=0);
 
    int16_t average();

    uint16_t average_pwr();

    bool write(uint16_t pos, int16_t value);

    int16_t read(uint16_t pos);

private:
    uint16_t m_size;
    //QVector<int16_t> m_Buffer;
    std::vector<int16_t> m_Buffer;

};
}
#endif 
