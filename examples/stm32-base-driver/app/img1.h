#ifndef IMG1_H
#define IMG1_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define IMG1_W 80
#define IMG1_H 64
#define IMG1_BYTES (IMG1_W * IMG1_H * 2u)
extern const uint8_t img1_rgb565[IMG1_BYTES];
#ifdef __cplusplus
}
#endif
#endif
