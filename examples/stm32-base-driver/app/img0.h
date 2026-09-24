#ifndef IMG0_H
#define IMG0_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define IMG0_W 80
#define IMG0_H 64
#define IMG0_BYTES (IMG0_W * IMG0_H * 2u)
extern const uint8_t img0_rgb565[IMG0_BYTES];
#ifdef __cplusplus
}
#endif
#endif
