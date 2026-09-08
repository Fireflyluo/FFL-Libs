/**
 * @file app_task.c
 * @brief 用四个仓库组件搭的最小应用示例。
 *
 * 组件 -> 用法：
 *   - ffl.osal      : 任务数组 + 事件 + 消息定时器。本任务由 osal 调度，
 *                     LED 心跳与统计上报都走 osal 的 reload 定时器触发事件。
 *   - ffl.sw_timer  : 基于时间轮的软件定时器。用 500ms 周期定时器调度
 *                     SC7A20 采样（到期回调里向 osal 任务置 EVENT_SAMPLE）。
 *   - ffl.sc7a20    : 加速度计。任务事件里调用 ffl_sc7a20_read_raw()
 *                     经 bsp 的 I2C1 transport（ffl.driver_port）读原始轴数据。
 *   - ffl.ringbuffer: 固定容量环形缓冲。每次采样把 6 字节原始数据写入，
 *                     统计事件里成块取出并记录字节数。
 *
 * 说明：SC7A20 未接入时传感器标记为不可用，采样槽仍写入全 0xFF 的占位
 * 记录，ringbuffer / sw_timer / osal 链路不依赖真实硬件即可工作；接入后
 * 只改 s_accel_addr 并在 README 核对 SDO 电平即可。
 */
#include "app_task.h"

#include "board.h"
#include "osal_event.h"
#include "osal_timer.h"
#include "type.h"

#include "ffl/ringbuffer.h"
#include "ffl/sc7a20.h"
#include "ffl/sw_timer.h"

#include "sc7a20_i2c_transport.h"
#include "stm32_time_ops.h"

#include <string.h>

/* OSAL 事件位（避免使用系统保留的最高位） */
#define APP_EVT_LED 0x0001u
#define APP_EVT_SAMPLE 0x0002u
#define APP_EVT_STATS 0x0004u

#define LED_HEARTBEAT_MS 500u /* osal reload 定时器：LED 心跳 */
#define SAMPLE_PERIOD_MS 500u /* sw_timer 周期：触发一次采样 */
#define STATS_PERIOD_MS 3000u /* osal reload 定时器：统计上报 */

#define RB_CAP_SAMPLES 64u  /* 环形缓冲容量（条） */
#define SAMPLE_REC_BYTES 6u /* 每条记录字节数（x/y/z int16 LE） */

/* 调试用计数器，可在调试器里观察 */
volatile uint32_t g_sample_ok_count;
volatile uint32_t g_sample_err_count;
volatile uint32_t g_drained_bytes;

static uint8_t s_task_id;

static uint8_t s_rb_pool[RB_CAP_SAMPLES * SAMPLE_REC_BYTES];
static ffl_ringbuffer_t s_rb;

static ffl_sw_timer_t s_sample_timer;

static ffl_sc7a20_device_t s_accel;
static uint8_t s_sensor_ok;

static void encode_sample(uint8_t rec[SAMPLE_REC_BYTES],
                          const ffl_sc7a20_raw_t *raw) {
  uint16_t ux;
  uint16_t uy;
  uint16_t uz;

  ux = (uint16_t)raw->x;
  uy = (uint16_t)raw->y;
  uz = (uint16_t)raw->z;
  rec[0] = (uint8_t)(ux & 0xFFu);
  rec[1] = (uint8_t)((ux >> 8) & 0xFFu);
  rec[2] = (uint8_t)(uy & 0xFFu);
  rec[3] = (uint8_t)((uy >> 8) & 0xFFu);
  rec[4] = (uint8_t)(uz & 0xFFu);
  rec[5] = (uint8_t)((uz >> 8) & 0xFFu);
}

static void read_one_sample(void) {
  uint8_t rec[SAMPLE_REC_BYTES];
  ffl_sc7a20_raw_t raw;
  int rc;

  rc = -1;
  if (s_sensor_ok != 0u) {
    rc = ffl_sc7a20_read_raw(&s_accel, &raw);
  }

  if (rc == 0) {
    encode_sample(rec, &raw);
    g_sample_ok_count++;
  } else {
    (void)memset(rec, 0xFF, sizeof(rec));
    g_sample_err_count++;
  }

  /* 环形缓冲总是写入，演示 put；满时自动丢弃尾部超出部分 */
  (void)ffl_ringbuffer_put(&s_rb, rec, sizeof(rec));
}

/* sw_timer 到期回调：任务上下文，安全地给 osal 任务置事件 */
static void sw_sample_expired(void *arg) {
  (void)arg;
  (void)osal_set_event(s_task_id, APP_EVT_SAMPLE);
}

static void app_task_init(uint8_t task_id) {
  ffl_sc7a20_config_t config;
  int rc;

  s_task_id = task_id;

  ffl_ringbuffer_init(&s_rb, s_rb_pool, sizeof(s_rb_pool));

  /* ---- ffl.sc7a20：bind 到 bsp 的 I2C1 transport + time ops ---- */
  rc = ffl_sc7a20_bind(&s_accel, bsp_sc7a20_transport(), bsp_time_ops(), NULL);
  if (rc == 0) {
    /* SC7A20 模块 SDO 接 GND -> 0x18；SDO 悬空/接高 -> 0x19 */
    rc = ffl_sc7a20_set_i2c_addr(&s_accel, FFL_SC7A20_DEFAULT_ADDR7_L);
  }
  if (rc == 0) {
    ffl_sc7a20_config_init(&config);
    config.range = FFL_SC7A20_RANGE_2G;
    config.odr = FFL_SC7A20_ODR_100HZ;
    rc = ffl_sc7a20_init(&s_accel, &config);
  }
  s_sensor_ok = (uint8_t)((rc == 0) ? 1u : 0u);

  /* ---- ffl.osal：任务级 reload 定时器 ---- */
  (void)osal_start_reload_timer(task_id, APP_EVT_LED, LED_HEARTBEAT_MS);
  (void)osal_start_reload_timer(task_id, APP_EVT_STATS, STATS_PERIOD_MS);

  /* ---- ffl.sw_timer：周期触发采样 ---- */
  (void)ffl_sw_timer_start(&s_sample_timer, SAMPLE_PERIOD_MS, SAMPLE_PERIOD_MS,
                           sw_sample_expired, NULL);
}

static uint16_t app_task_event(uint8_t task_id, uint16_t events) {
  uint8_t drain[RB_CAP_SAMPLES * SAMPLE_REC_BYTES];

  (void)task_id;

  if ((events & APP_EVT_LED) != 0u) {
    board_led_toggle();
    events &= (uint16_t)~APP_EVT_LED;
  }

  if ((events & APP_EVT_SAMPLE) != 0u) {
    read_one_sample();
    events &= (uint16_t)~APP_EVT_SAMPLE;
  }

  if ((events & APP_EVT_STATS) != 0u) {
    /* 演示 ringbuffer get：成块取出，统计这次清空了多少字节 */
    g_drained_bytes +=
        (uint32_t)ffl_ringbuffer_get(&s_rb, drain, sizeof(drain));
    events &= (uint16_t)~APP_EVT_STATS;
  }

  return events;
}

void app_task_register(void) {
  osal_add_Task(app_task_init, app_task_event, 1u);
}
