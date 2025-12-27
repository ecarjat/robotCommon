#include "framing_cobs.h"

size_t robot_cobs_encode(const uint8_t *input, size_t length, uint8_t *output, size_t output_size)
{
    if (output_size == 0U)
    {
        return 0U;
    }

    size_t read_index = 0U;
    size_t write_index = 1U;
    size_t code_index = 0U;
    uint8_t code = 1U;

    while (read_index < length)
    {
        if (write_index >= output_size)
        {
            return 0U;
        }

        uint8_t byte = input[read_index++];
        if (byte == 0U)
        {
            output[code_index] = code;
            code = 1U;
            code_index = write_index++;
        }
        else
        {
            output[write_index++] = byte;
            code++;
            if (code == 0xFFU)
            {
                if (write_index >= output_size)
                {
                    return 0U;
                }
                output[code_index] = code;
                code = 1U;
                code_index = write_index++;
            }
        }
    }

    if (write_index > output_size)
    {
        return 0U;
    }

    output[code_index] = code;
    return write_index;
}

size_t robot_cobs_decode(const uint8_t *input, size_t length, uint8_t *output, size_t output_size)
{
    if (output_size == 0U)
    {
        return 0U;
    }

    size_t read_index = 0U;
    size_t write_index = 0U;

    while (read_index < length)
    {
        uint8_t code = input[read_index++];
        if (code == 0U)
        {
            return 0U;
        }

        for (uint8_t i = 1U; i < code; ++i)
        {
            if (read_index >= length || write_index >= output_size)
            {
                return 0U;
            }
            output[write_index++] = input[read_index++];
        }

        if (code < 0xFFU && read_index < length)
        {
            if (write_index >= output_size)
            {
                return 0U;
            }
            output[write_index++] = 0U;
        }
    }

    return write_index;
}
