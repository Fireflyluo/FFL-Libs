/**
 * @file alarm_task.c
 * @brief 串口固定长度命令：防空警报鸣放 + 图切换请求。
 *
 * 鸣放时序用 OSAL 1s 粒度计数（空袭/预先/解除均为秒级），
 * 停止命令随时可打断。蜂鸣器 PB6 低电平响。
 */
#include <stdio.h>
#include <string.h>

#include "alarm_task.h"
#include "lcd_ui.h"
#include "board.h"
#include "osal_event.h"
#include "osal_tasks.h"
#include "osal_timer.h"

#define EVT_TICK 0x0001u
#define EVT_CMD 0x0002u
#define TICK_MS 20u

enum {
  ALARM_IDLE = 0,
  ALARM_AIR,   /* 鸣6 停6 ×15 */
  ALARM_PRE,   /* 鸣36 停24 ×3 */
  ALARM_CLEAR  /* 连续 180s */
};

static uint8_t s_tid;
static uint8_t s_mode = ALARM_IDLE;
static uint8_t s_phase;     /* 0=鸣 1=停 */
static uint16_t s_rem_ms;   /* 当前相位剩余（100ms 单位） */
static uint8_t s_cycles_left;
static uint8_t s_pending_img; /* 0=无 1..3 */
#define CMD_BODY_LEN 8u

static uint8_t s_command[CMD_BODY_LEN];
static uint8_t s_command_n;
static uint8_t s_ignore_next_lf;

uint8_t alarm_is_active(void) { return (uint8_t)(s_mode != ALARM_IDLE); }
uint8_t alarm_pending_image(void) { return s_pending_img; }
void alarm_clear_pending_image(void) { s_pending_img = 0u; }

static uint8_t s_wail;

static uint8_t tri8(uint8_t p) {
  return (p < 128u) ? (uint8_t)(p * 2u) : (uint8_t)(255u - (uint16_t)(p - 128u) * 2u);
}

static void tone_update(void) {
  uint16_t hz;
  uint8_t t;
  if (s_phase != 0u || s_mode == ALARM_IDLE) {
    board_buzzer_off();
    return;
  }
  switch (s_mode) {
    case ALARM_AIR:
    case ALARM_PRE:
      /* 统一为慢速「呜——」350–900Hz；AIR 仅时序更急（6s 开/关） */
      s_wail = (uint8_t)(s_wail + 2u);
      t = tri8(s_wail);
      hz = (uint16_t)(350u + ((uint16_t)t * 550u) / 255u);
      break;
    case ALARM_CLEAR:
      hz = 1000u;
      break;
    default:
      board_buzzer_off();
      return;
  }
  board_buzzer_tone(hz);
}

static void alarm_stop(const char *why) {
  s_mode = ALARM_IDLE;
  s_phase = 0u;
  s_rem_ms = 0u;
  s_cycles_left = 0u;
  board_buzzer_off();
  lcd_ui_set_mode(LCD_UI_IDLE);
  lcd_ui_log("STOP -> gray, buzzer off");
  printf("[alarm] stop (%s)\n", why ? why : "-");
}

static void alarm_start(uint8_t mode) {
  s_mode = mode;
  s_phase = 0u; /* 先鸣 */
  s_wail = 0u;
  tone_update();
  switch (mode) {
    case ALARM_AIR:
      s_rem_ms = (uint16_t)(6000u / TICK_MS); /* 6s */
      s_cycles_left = 15u;
      lcd_ui_set_mode(LCD_UI_AIR);
      lcd_ui_log("ARDA wail 350-900Hz on6s off6s x15");
      printf("[alarm] AIR slow-wail 鸣6s停6s x15\n");
      break;
    case ALARM_PRE:
      s_rem_ms = (uint16_t)(36000u / TICK_MS); /* 36s */
      s_cycles_left = 3u;
      lcd_ui_set_mode(LCD_UI_PRE);
      lcd_ui_log("PREA wail 350-900Hz on36s off24s x3");
      printf("[alarm] PRE slow-wail 鸣36s停24s x3\n");
      break;
    case ALARM_CLEAR:
      s_rem_ms = (uint16_t)(180000u / TICK_MS); /* 180s */
      s_cycles_left = 1u;
      lcd_ui_set_mode(LCD_UI_CLEAR);
      lcd_ui_log("ACLR steady 1000Hz 180s");
      printf("[alarm] CLEAR steady 180s\n");
      break;
    default:
      alarm_stop("bad mode");
      break;
  }
}

static void handle_command(const uint8_t command[CMD_BODY_LEN]) {
  if (memcmp(command, "cmd:STOP", CMD_BODY_LEN) == 0) {
    alarm_stop("cmd");
    return;
  }
  if (memcmp(command, "cmd:ARDA", CMD_BODY_LEN) == 0) {
    alarm_start(ALARM_AIR);
    return;
  }
  if (memcmp(command, "cmd:PREA", CMD_BODY_LEN) == 0) {
    alarm_start(ALARM_PRE);
    return;
  }
  if (memcmp(command, "cmd:ACLR", CMD_BODY_LEN) == 0) {
    alarm_start(ALARM_CLEAR);
    return;
  }
  if (memcmp(command, "cmd:IMG1", CMD_BODY_LEN) == 0) {
    s_pending_img = 1u;
    lcd_ui_log("IMG1 slot0 base=0x000000");
    lcd_ui_show_image(0u);
    printf("[alarm] cmd -> img1\n");
    return;
  }
  if (memcmp(command, "cmd:IMG2", CMD_BODY_LEN) == 0) {
    s_pending_img = 2u;
    lcd_ui_log("IMG2 slot1 base=0x028000");
    lcd_ui_show_image(1u);
    printf("[alarm] cmd -> img2\n");
    return;
  }
  if (memcmp(command, "cmd:IMG3", CMD_BODY_LEN) == 0) {
    s_pending_img = 3u;
    lcd_ui_log("IMG3 slot2 base=0x050000");
    lcd_ui_show_image(2u);
    printf("[alarm] cmd -> img3\n");
    return;
  }
  printf("[alarm] unknown frame\n");
}

static void command_reset_from(uint8_t byte) {
  s_command_n = 0u;
  if (byte == (uint8_t)'c') {
    s_command[s_command_n++] = byte;
  }
}

void alarm_feed_byte(uint8_t b) {
  static const uint8_t prefix[] = "cmd:";

  if (s_ignore_next_lf != 0u) {
    s_ignore_next_lf = 0u;
    if (b == (uint8_t)'\n') {
      return;
    }
  }
  if (b == (uint8_t)'\r' || b == (uint8_t)'\n') {
    if (s_command_n == CMD_BODY_LEN) {
      handle_command(s_command);
    }
    command_reset_from(0u);
    if (b == (uint8_t)'\r') {
      s_ignore_next_lf = 1u;
    }
    return;
  }

  if (s_command_n < sizeof(prefix) - 1u) {
    if (b == prefix[s_command_n]) {
      s_command[s_command_n++] = b;
    } else {
      command_reset_from(b);
    }
    return;
  }

  if (s_command_n < CMD_BODY_LEN) {
    s_command[s_command_n++] = b;
  } else {
    command_reset_from(b);
  }
}

static void poll_uart(void) {
  int ch;
  while ((ch = board_uart_getc()) >= 0) {
    alarm_feed_byte((uint8_t)ch);
  }
}

/** 每 20ms：扫频 + 相位时序 */
static void alarm_tick(void) {
  if (s_mode == ALARM_IDLE) {
    return;
  }
  tone_update();
  if (s_rem_ms > 0u) {
    s_rem_ms--;
  }
  if (s_rem_ms > 0u) {
    return;
  }
  if (s_phase == 0u) {
    board_buzzer_off();
    s_phase = 1u;
    if (s_mode == ALARM_AIR) {
      s_rem_ms = (uint16_t)(6000u / TICK_MS);
    } else if (s_mode == ALARM_PRE) {
      s_rem_ms = (uint16_t)(24000u / TICK_MS);
    } else if (s_mode == ALARM_CLEAR) {
      alarm_stop("clear done");
    }
    return;
  }
  if (s_cycles_left > 0u) {
    s_cycles_left--;
  }
  if (s_cycles_left == 0u) {
    alarm_stop("cycles done");
    return;
  }
  s_phase = 0u;
  s_wail = 0u;
  if (s_mode == ALARM_AIR) {
    s_rem_ms = (uint16_t)(6000u / TICK_MS);
  } else if (s_mode == ALARM_PRE) {
    s_rem_ms = (uint16_t)(36000u / TICK_MS);
  }
  tone_update();
}

static void alarm_init(uint8_t task_id) {
  s_tid = task_id;
  s_command_n = 0u;
  s_ignore_next_lf = 0u;
  s_pending_img = 0u;
  alarm_stop("boot");
  (void)osal_start_reload_timer(task_id, EVT_TICK, TICK_MS);
  printf("[alarm] cmds cmd:ARDA\\n cmd:PREA\\n cmd:ACLR\\n cmd:STOP\\n cmd:IMG1..3\\n\n");
}

static uint16_t alarm_event(uint8_t task_id, uint16_t events) {
  (void)task_id;
  if (events & EVT_TICK) {
    poll_uart();
    alarm_tick();
    events &= (uint16_t)~EVT_TICK;
  }
  return events;
}

void alarm_task_register(void) { osal_add_Task(alarm_init, alarm_event, 2u); }
