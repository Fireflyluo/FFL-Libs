/**
 * @file font6x8.h
 * @brief 6x8 点阵 ASCII（32..126），LCD 底栏日志用。
 */
#ifndef FONT6X8_H
#define FONT6X8_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FONT6X8_W 6
#define FONT6X8_ROWS 8

/** glyph[8]：每字节一行，bit0..5 为左→右像素 */
const uint8_t *font6x8_glyph(char ch);

#ifdef __cplusplus
}
#endif

#endif
