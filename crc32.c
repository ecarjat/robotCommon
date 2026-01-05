#include "crc32.h"

#include <string.h>

#define CRC32_POLY 0xEDB88320UL

#ifndef ROBOT_USE_HW_CRC
#define ROBOT_USE_HW_CRC 0
#endif

#if defined(__has_include)
#if __has_include("stm32h7xx_hal.h")
#include "stm32h7xx_hal.h"
#define ROBOT_HAS_STM32_HAL 1
extern CRC_HandleTypeDef hcrc;
#endif
#endif

uint32_t robot_crc32(const uint8_t *data, size_t length)
{
#if ROBOT_USE_HW_CRC && defined(ROBOT_HAS_STM32_HAL) && defined(HAL_CRC_MODULE_ENABLED)
    if (data != NULL && length > 0U)
    {
        /* Use hardware CRC (polynomial 0x04C11DB7) with byte input inversion and output inversion to match the
         * reflected software CRC32/ISO (poly 0xEDB88320, refin/refout, init/xor 0xFFFFFFFF). */
        uint32_t prev_init = hcrc.Init.InitValue;
        uint32_t prev_in   = hcrc.Init.InputDataInversionMode;
        uint32_t prev_out  = hcrc.Init.OutputDataInversionMode;

        hcrc.Init.InitValue                = 0xFFFFFFFFU;
        hcrc.Init.InputDataInversionMode   = CRC_INPUTDATA_INVERSION_BYTE;
        hcrc.Init.OutputDataInversionMode  = CRC_OUTPUTDATA_INVERSION_ENABLE;

        if (HAL_CRC_Init(&hcrc) == HAL_OK)
        {
            const size_t word_count = (length + 3U) / 4U;
            /* Max frame decoded size is small (<260 bytes), so a fixed buffer is fine. */
            uint32_t words[70] = {0};
            if (word_count <= (sizeof(words) / sizeof(words[0])))
            {
                memcpy(words, data, length);
                uint32_t crc_hw = HAL_CRC_Calculate(&hcrc, words, (uint32_t)word_count) ^ 0xFFFFFFFFUL;

                /* Restore previous CRC config for any other users. */
                hcrc.Init.InitValue               = prev_init;
                hcrc.Init.InputDataInversionMode  = prev_in;
                hcrc.Init.OutputDataInversionMode = prev_out;
                (void)HAL_CRC_Init(&hcrc);

                return crc_hw;
            }
        }

        /* Restore configuration if hardware path failed. */
        hcrc.Init.InitValue               = prev_init;
        hcrc.Init.InputDataInversionMode  = prev_in;
        hcrc.Init.OutputDataInversionMode = prev_out;
        (void)HAL_CRC_Init(&hcrc);
    }
#endif
    uint32_t crc = 0xFFFFFFFFUL;
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
