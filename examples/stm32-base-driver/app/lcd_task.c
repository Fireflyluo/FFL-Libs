/**
 * @file lcd_task.c
 * @brief LCD 界面：警报色块 / 手动切图 / 底栏日志。无自动轮播。
 */
#include <stdio.h>
#include <string.h>

#include "alarm_task.h"
#include "lcd_ui.h"
#include "osal_event.h"
#include "osal_tasks.h"
#include "st7735.h"
#include "w25q.h"

uint8_t flash_task_ready(void);
uint32_t flash_task_img_addr(uint8_t idx);

static char s_log[80];

static void paint_status(uint16_t rgb) {
  st7735_fill_color(rgb);
  st7735_log_line(s_log);
}

void lcd_ui_log(const char *msg) {
  if (msg == NULL) {
    msg = "";
  }
  strncpy(s_log, msg, sizeof(s_log) - 1u);
  s_log[sizeof(s_log) - 1u] = 0;
  st7735_log_line(s_log);
}

void lcd_ui_set_mode(uint8_t mode) {
  uint16_t rgb;
  switch (mode) {
    case LCD_UI_AIR:
      rgb = 0xF800u; /* 红 */
      break;
    case LCD_UI_PRE:
      rgb = 0xFFE0u; /* 黄 */
      break;
    case LCD_UI_CLEAR:
      rgb = 0x07E0u; /* 绿 */
      break;
    case LCD_UI_IMG:
      return;
    case LCD_UI_IDLE:
    default:
      rgb = 0x8410u; /* 灰 */
      break;
  }
  paint_status(rgb);
}

void lcd_ui_show_image(uint8_t slot) {
  uint32_t base;
  uint8_t *row;
  uint16_t y;

  if (slot > 2u) {
    slot = 0u;
  }
  if (!flash_task_ready()) {
    lcd_ui_log("img not ready");
    lcd_ui_set_mode(LCD_UI_IDLE);
    return;
  }
  base = flash_task_img_addr(slot);
  row = st7735_line_buf();
  for (y = 0; y < LCD_H; y++) {
    if (y + 10u >= LCD_H) {
      break;
    }
    (void)w25q_read(base + (uint32_t)y * LCD_W * 2u, row, LCD_W * 2u);
    st7735_draw_row(y, row);
  }
  st7735_log_line(s_log);
  printf("[lcd] img%u base=0x%06lX\n", (unsigned)(slot + 1u),
         (unsigned long)base);
}

static void lcd_init_task(uint8_t task_id) {
  (void)task_id;
  st7735_init();
  snprintf(s_log, sizeof(s_log), "ready %dx%d", LCD_W, LCD_H);
  lcd_ui_set_mode(LCD_UI_IDLE);
  printf("[lcd] UI gray/RGB alert/IMGx + log bar\n");
}

static uint16_t lcd_event(uint8_t task_id, uint16_t events) {
  uint8_t img;
  (void)task_id;
  img = alarm_pending_image();
  if (img != 0u) {
    alarm_clear_pending_image();
    snprintf(s_log, sizeof(s_log), "IMG%u", img);
    lcd_ui_show_image((uint8_t)(img - 1u));
  }
  return events;
}

void lcd_task_register(void) {
  osal_add_Task(lcd_init_task, lcd_event, 1u);
}
