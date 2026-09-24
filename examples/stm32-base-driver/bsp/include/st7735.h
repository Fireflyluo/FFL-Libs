/**
 * @file st7735.h
 * @brief ST7789 12pin 驱动接口（API 名沿用 st7735_* 兼容 app）。
 *
 * 接线（STM32-LORA 网名 → 12pin FPC，见 硬件映射表.md）：
 *   S1C/MOSI1 → SCL/SDA = PA5/PA7  SPI1
 *   S1N       → CSX     = PA4
 *   DBSY      → DCX     = PB1
 *   DRST      → RESX    = PB0
 *   TXEN      → LEDK    = PB12（见下方背光说明）
 *
 * 逻辑分辨率：横屏 LCD_W×LCD_H = 320×240（面板物理 240×320）。
 */
#ifndef ST7735_H
#define ST7735_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 横屏：宽 320（长边水平）、高 240 */
#define LCD_W 320
#define LCD_H 240

void st7735_init(void);
void st7735_bus_acquire(void);
void st7735_draw_rgb565(const uint8_t *pixels, uint32_t len);
void st7735_draw_row(uint16_t y, const uint8_t *row);
uint32_t st7735_draw_rgb565_timed(const uint8_t *pixels, uint32_t len);

/**
 * 自检色条：横屏时按 y 画横条，自上而下
 * 红-绿-蓝-白-黄-青-品红。返回耗时微秒。
 */
uint32_t st7735_selftest_bars(void);

/** 整屏刷标准 RGB565（驱动内做面板反色补偿）。 */
void st7735_fill_color(uint16_t rgb565);

/** 底栏日志条 + 6x8 文本 */
void st7735_log_line(const char *text);

/** 控制脚电平：bit0=CS bit1=DC bit2=RES bit3=BL */
uint32_t st7735_gpio_snapshot(void);

/** 驱动内一行缓冲（LCD_W*2），供刷 Flash 图时复用，避免 app 再占 BSS */
uint8_t *st7735_line_buf(void);

#ifdef __cplusplus
}
#endif

#endif
