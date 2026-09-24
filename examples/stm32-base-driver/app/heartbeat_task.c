/**
 * @file heartbeat_task.c
 * @brief LED 心跳；空闲时每 60s「嘀—嘀」；警报进行中让出蜂鸣器。
 */
#include "alarm_task.h"
#include "board.h"
#include "osal_event.h"
#include "osal_timer.h"
#include "osal_tasks.h"
#include "ffl/sw_timer.h"

#define EVT_LED 0x0001u
#define EVT_BEEP 0x0002u

static uint8_t s_tid;
static ffl_sw_timer_t s_beep;
static uint8_t s_beep_stage; /* 0 空闲 1 第一声后间隙 2 第二声后间隙 */

static void sw_beep_step(void *arg) {
  (void)arg;
  board_buzzer_off();
  if (s_beep_stage == 1u) {
    /* 第一声结束，120ms 后第二声 */
    (void)ffl_sw_timer_start(&s_beep, 120u, 0u, sw_beep_step, NULL);
    s_beep_stage = 2u;
    return;
  }
  if (s_beep_stage == 2u) {
    board_buzzer_on();
    (void)ffl_sw_timer_start(&s_beep, 60u, 0u, sw_beep_step, NULL);
    s_beep_stage = 3u;
    return;
  }
  s_beep_stage = 0u; /* 第二声结束 */
}

static void heartbeat_init(uint8_t task_id) {
  s_tid = task_id;
  board_buzzer_off();
  (void)osal_start_reload_timer(task_id, EVT_LED, 500u);
  (void)osal_start_reload_timer(task_id, EVT_BEEP, 60000u);
}

static uint16_t heartbeat_event(uint8_t task_id, uint16_t events) {
  (void)task_id;
  if (events & EVT_LED) {
    board_led_toggle();
    events &= (uint16_t)~EVT_LED;
  }
  if (events & EVT_BEEP) {
    /* 警报占用蜂鸣器时不抢 */
    if (s_beep_stage == 0u && !alarm_is_active()) {
      board_buzzer_on();
      s_beep_stage = 1u;
      (void)ffl_sw_timer_start(&s_beep, 60u, 0u, sw_beep_step, NULL);
    }
    events &= (uint16_t)~EVT_BEEP;
  }
  return events;
}

void heartbeat_task_register(void) {
  osal_add_Task(heartbeat_init, heartbeat_event, 1u);
}
