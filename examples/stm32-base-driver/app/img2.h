#ifndef IMG2_H
#define IMG2_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define IMG2_W 80
#define IMG2_H 64
#define IMG2_BYTES (IMG2_W * IMG2_H * 2u)
extern const uint8_t img2_rgb565[IMG2_BYTES];
#ifdef __cplusplus
}
#endif
#endif
