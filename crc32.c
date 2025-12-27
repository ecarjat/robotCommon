#include "crc32.h"

#define CRC32_POLY 0xEDB88320UL

uint32_t robot_crc32(const uint8_t *data, size_t length, uint32_t seed)
{
    uint32_t crc = seed ^ 0xFFFFFFFFUL;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            uint32_t mask = -(crc & 1UL);
            crc = (crc >> 1) ^ (CRC32_POLY & mask);
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}
