#ifndef ROBOT_FRAMING_COBS_H
#define ROBOT_FRAMING_COBS_H

#include <stddef.h>
#include <stdint.h>

size_t robot_cobs_encode(const uint8_t *input, size_t length, uint8_t *output, size_t output_size);
size_t robot_cobs_decode(const uint8_t *input, size_t length, uint8_t *output, size_t output_size);

#endif /* ROBOT_FRAMING_COBS_H */
