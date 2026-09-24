/**
 * @file lcd_ui.h
 * @brief LCD 状态显示：警报色块 / 图 / 底栏日志。
 */
#ifndef APP_LCD_UI_H
#define APP_LCD_UI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
  LCD_UI_IDLE = 0,
  LCD_UI_AIR = 1,
  LCD_UI_PRE = 2,
  LCD_UI_CLEAR = 3,
  LCD_UI_IMG = 4
};

void lcd_ui_set_mode(uint8_t mode);
void lcd_ui_show_image(uint8_t slot);
void lcd_ui_log(const char *msg);

#ifdef __cplusplus
}
#endif

#endif
