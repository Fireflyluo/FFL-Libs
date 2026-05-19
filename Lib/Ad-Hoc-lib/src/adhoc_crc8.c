#include "adhoc_crc8.h"

uint8_t adhoc_crc8_compute(const uint8_t *data, uint16_t len)
{
    uint16_t index;
    uint8_t bit_index;
    uint8_t crc = ADHOC_CRC8_INIT;
    uint8_t current;

    if (data == 0)
    {
        return ADHOC_CRC8_INIT ^ ADHOC_CRC8_XOROUT;
    }

    for (index = 0u; index < len; ++index)
    {
        current = data[index];
        crc ^= current;
        for (bit_index = 0u; bit_index < 8u; ++bit_index)
        {
            if ((crc & 0x80u) != 0u)
            {
                crc = (uint8_t)((crc << 1) ^ ADHOC_CRC8_POLY);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return (uint8_t)(crc ^ ADHOC_CRC8_XOROUT);
}

uint8_t adhoc_crc8_frame(const uint8_t frame[32])
{
    if (frame == 0)
    {
        return ADHOC_CRC8_INIT ^ ADHOC_CRC8_XOROUT;
    }
    return adhoc_crc8_compute(frame, 31u);
}

int adhoc_crc8_verify_frame(const uint8_t frame[32])
{
    if (frame == 0)
    {
        return 0;
    }
    return adhoc_crc8_frame(frame) == frame[31] ? 1 : 0;
}

void adhoc_crc8_write_frame(uint8_t frame[32])
{
    if (frame == 0)
    {
        return;
    }
    frame[31] = adhoc_crc8_frame(frame);
}
