#ifndef ROBOT_CRC32_H
#define ROBOT_CRC32_H

#include <stddef.h>
#include <stdint.h>

uint32_t robot_crc32(const uint8_t *data, size_t length, uint32_t seed);

#endif /* ROBOT_CRC32_H */
