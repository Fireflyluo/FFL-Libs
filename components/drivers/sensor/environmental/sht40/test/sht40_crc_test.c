#include "sht40_core.h"

#include <stdio.h>

static int test_accepts_valid_frame(void)
{
    const uint8_t frame[6] = {0xBEu, 0xEFu, 0x92u, 0xBEu, 0xEFu, 0x92u};
    sht40_sample_t sample;

    return sht40_core_read_sample_parse(frame, &sample) == 0 ? 0 : 1;
}

static int test_rejects_invalid_crc(void)
{
    uint8_t frame[6] = {0xBEu, 0xEFu, 0x92u, 0xBEu, 0xEFu, 0x92u};
    sht40_sample_t sample;

    frame[5] ^= 0x01u;
    return sht40_core_read_sample_parse(frame, &sample) != 0 ? 0 : 1;
}

int main(void)
{
    const int failed = test_accepts_valid_frame() + test_rejects_invalid_crc();

    if (failed != 0) {
        fprintf(stderr, "sht40 CRC tests failed: %d\n", failed);
    }
    return failed;
}
