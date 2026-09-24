/**
 * @file sc7a20_task.c
 * @brief SC7A20 采样任务：驱动 facade + ports I2C + ringbuffer 统计。
 *
 * 无传感器时 sensor_ok=0，仍写 0xFF 占位，验证调度链路（Task4）。
 */
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "board_i2c.h"
#include "ffl/ringbuffer.h"
#include "ffl/sc7a20.h"
#include "ffl/sw_timer.h"
#include "ffl_port_stm32f1_driver_port.h"
#include "ffl_port_stm32f1_sc7a20.h"
#include "osal_event.h"
#include "osal_tasks.h"
#include "osal_timer.h"

#define EVT_SAMPLE 0x0001u
#define EVT_STATS 0x0002u

#define SAMPLE_MS 500u
#define STATS_MS 3000u
#define RB_CAP 64u
#define REC_LEN 6u

volatile uint32_t g_sample_ok;
volatile uint32_t g_sample_err;
volatile uint32_t g_drained;
volatile int16_t g_ax, g_ay, g_az;
static uint8_t s_tid;
static uint8_t s_sensor_ok;
static uint8_t s_pool[RB_CAP * REC_LEN];
static ffl_ringbuffer_t s_rb;
static ffl_sw_timer_t s_timer;
static ffl_sc7a20_device_t s_dev;
static ffl_stm32f1_i2c_ctx_t s_i2c_ctx;
static ffl_transport_t s_i2c_transport;
static int s_last_sample_rc;

static void pack(uint8_t rec[REC_LEN], const ffl_sc7a20_raw_t *r) {
  rec[0] = (uint8_t)r->x;
  rec[1] = (uint8_t)((uint16_t)r->x >> 8);
  rec[2] = (uint8_t)r->y;
  rec[3] = (uint8_t)((uint16_t)r->y >> 8);
  rec[4] = (uint8_t)r->z;
  rec[5] = (uint8_t)((uint16_t)r->z >> 8);
}

static void do_sample(void) {
  uint8_t rec[REC_LEN];
  ffl_sc7a20_raw_t raw;
  int rc = -1;

  if (s_sensor_ok) {
    rc = ffl_sc7a20_read_raw(&s_dev, &raw);
  }
  s_last_sample_rc = rc;
  if (rc == 0) {
    pack(rec, &raw);
    g_ax = raw.x;
    g_ay = raw.y;
    g_az = raw.z;
    g_sample_ok++;
  } else {
    memset(rec, 0xFF, REC_LEN);
    g_sample_err++;
  }
  (void)ffl_ringbuffer_put(&s_rb, rec, REC_LEN);
}

static void sw_expired(void *arg) {
  (void)arg;
  (void)osal_set_event(s_tid, EVT_SAMPLE);
}

static void sc7a20_init(uint8_t task_id) {
  ffl_sc7a20_config_t cfg;
  int rc;

  s_tid = task_id;
  ffl_ringbuffer_init(&s_rb, s_pool, sizeof(s_pool));

  s_i2c_ctx.hi2c = board_i2c1_handle();
  s_i2c_ctx.timeout_ms = 50u;
  rc = ffl_stm32f1_i2c_transport_setup(&s_i2c_transport, &s_i2c_ctx,
                                        FFL_SC7A20_DEFAULT_ADDR7_L);
  if (rc == 0) {
    rc = ffl_stm32f1_sc7a20_bind(&s_dev, &s_i2c_transport,
                                  FFL_SC7A20_DEFAULT_ADDR7_L);
  }
  if (rc == 0) {
    ffl_sc7a20_config_init(&cfg);
    cfg.range = FFL_SC7A20_RANGE_2G;
    cfg.odr = FFL_SC7A20_ODR_100HZ;
    rc = ffl_sc7a20_init(&s_dev, &cfg);
  }
  s_sensor_ok = (uint8_t)(rc == 0);
  s_last_sample_rc = rc;
  printf("[sc7a20] task init rc=%d ok=%u\n", rc, s_sensor_ok);

  (void)osal_start_reload_timer(task_id, EVT_STATS, STATS_MS);
  (void)ffl_sw_timer_start(&s_timer, SAMPLE_MS, SAMPLE_MS, sw_expired, NULL);
}

static uint16_t sc7a20_event(uint8_t task_id, uint16_t events) {
  uint8_t drain[RB_CAP * REC_LEN];
  (void)task_id;

  if (events & EVT_SAMPLE) {
    do_sample();
    events &= (uint16_t)~EVT_SAMPLE;
  }
  if (events & EVT_STATS) {
    uint8_t who = 0;
    int wrc = -1;
    g_drained += (uint32_t)ffl_ringbuffer_get(&s_rb, drain, sizeof(drain));
    if (s_sensor_ok) {
      wrc = ffl_sc7a20_who_am_i(&s_dev, &who);
    }
    printf("[sc7a20] ok=%lu err=%lu drained=%lu raw=(%d,%d,%d) sample_rc=%d who_rc=%d id=%02X\n",
           (unsigned long)g_sample_ok, (unsigned long)g_sample_err,
           (unsigned long)g_drained, (int)g_ax, (int)g_ay, (int)g_az,
           s_last_sample_rc, wrc, who);
    events &= (uint16_t)~EVT_STATS;
  }
  return events;
}

void sc7a20_task_register(void) {
  osal_add_Task(sc7a20_init, sc7a20_event, 1u);
}
