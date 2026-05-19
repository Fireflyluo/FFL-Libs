#ifndef ADHOC_CRC8_H
#define ADHOC_CRC8_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADHOC_CRC8_POLY 0x07u
#define ADHOC_CRC8_INIT 0x00u
#define ADHOC_CRC8_XOROUT 0x00u

uint8_t adhoc_crc8_compute(const uint8_t *data, uint16_t len);
uint8_t adhoc_crc8_frame(const uint8_t frame[32]);
int adhoc_crc8_verify_frame(const uint8_t frame[32]);
void adhoc_crc8_write_frame(uint8_t frame[32]);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_CRC8_H */
