/**
 * @file app_task.c
 * @brief 业务任务：把多个 ffl 组件串成一条可运行的数据流。
 *
 * ========================= 数据流（看懂这张图即可） =========================
 *
 *   SysTick 1ms
 *     ├─ osal_update_timers()      → LED 500ms、STATS 3s 到期 → 置事件位
 *     └─ ffl_sw_timer_tick_isr()   → 时间轮前进
 *
 *   sw_timer 500ms 到期（主循环 process 里回调）
 *     └─ sw_sample_expired()       → osal_set_event(EVT_SAMPLE)
 *
 *   app_task_event()
 *     ├─ EVT_LED     → board_led_toggle()
 *     ├─ EVT_SAMPLE  → ffl_sc7a20_read_raw() → ringbuffer_put(6B)
 *     └─ EVT_STATS   → ringbuffer_get() + printf + 蜂鸣短鸣
 *
 * ========================= 本文件用到的仓库组件 =============================
 *
 *   ffl.osal       任务 + 事件 + reload 定时器     [components/runtime/osal]
 *   ffl.sw_timer   500ms 采样、80ms 蜂鸣脉宽      [components/foundation/sw-timer]
 *   ffl.ringbuffer 采样记录缓冲                   [components/foundation/ringbuffer]
 *   ffl.sc7a20     加速度计 facade                [components/drivers/.../sc7a20]
 *
 * ========================= 有 port / 无 port ================================
 *
 *   有 port（本板）：
 *     board_i2c1_init_pb89() 已配好 I2C1；
 *     ffl_stm32f1_i2c_transport_setup() 把 handle 注入 ports 的 xfer；
 *     再 ffl_stm32f1_sc7a20_bind()。
 *
 *   无传感器（Task 4）：
 *     bind/init 失败 → s_sensor_ok=0，EVT_SAMPLE 仍写 0xFF 占位进 ringbuffer，
 *     用来证明 OSAL/sw_timer/ringbuffer 链路不依赖真实硬件。
 *
 * 串口：USART1 → COM4 @115200，打印 [boot]/[sc7a20]/[stats]。
 * 文档：docs/tasks/01~04.md
 */
#include "app_task.h"

#include <stdio.h>

#include "board.h"
#include "osal_event.h"
#include "osal_timer.h"
#include "type.h"

#include "ffl/ringbuffer.h"
#include "ffl/sc7a20.h"
#include "ffl/sw_timer.h"

#include "board_i2c.h"
#include "ffl_port_stm32f1_driver_port.h"
#include "ffl_port_stm32f1_sc7a20.h"

#include <string.h>

/* ---- OSAL 事件位：在 app_task_event() 里轮询处理（勿用系统保留最高位） ---- */
#define APP_EVT_LED 0x0001u    /* LED 翻转 */
#define APP_EVT_SAMPLE 0x0002u /* 读一次加速度并入环 */
#define APP_EVT_STATS 0x0004u  /* 从环中取出并打印统计 */

/* ---- 周期参数（毫秒）---- */
#define LED_HEARTBEAT_MS 500u /* OSAL reload：心跳 */
#define SAMPLE_PERIOD_MS 500u /* sw_timer 周期：采样 */
#define STATS_PERIOD_MS 3000u /* OSAL reload：统计 */
#define BEEP_MS 80u           /* sw_timer 单次：蜂鸣脉宽 */

#define RB_CAP_SAMPLES 64u  /* ringbuffer 容量（条） */
#define SAMPLE_REC_BYTES 6u /* 记录格式：int16 x,y,z 小端 */

/* ---- 调试观察点（可挂在调试器 Watch）---- */
volatile uint32_t g_sample_ok_count;  /* 成功读芯片次数 */
volatile uint32_t g_sample_err_count; /* 失败或无传感器时的占位次数 */
volatile uint32_t g_drained_bytes;    /* STATS 从 ringbuffer 取出的总字节 */
volatile int16_t g_last_x, g_last_y, g_last_z;           /* 最近 raw */
volatile int32_t g_last_mg_x, g_last_mg_y, g_last_mg_z; /* 最近 milli-g */
volatile uint8_t g_who_am_i;          /* SC7A20 期望 0x11 */
volatile int32_t g_selftest_rc;       /* 上电驱动自检：0=通过 */

static uint8_t s_task_id; /* OSAL 分配的任务 id，供 set_event 使用 */

/* ringbuffer 内存池 + 控制块（Task 1） */
static uint8_t s_rb_pool[RB_CAP_SAMPLES * SAMPLE_REC_BYTES];
static ffl_ringbuffer_t s_rb;

/* 软定时器实例：采样周期 + 蜂鸣脉宽（Task 1） */
static ffl_sw_timer_t s_sample_timer;
static ffl_sw_timer_t s_beep_timer;

/* 加速度计设备对象 + 是否可用（Task 3 / Task 4） */
static ffl_sc7a20_device_t s_accel;
static uint8_t s_sensor_ok;

/**
 * SC7A20 facade 上电自检（可选）：who_am_i → read_raw/g → 量程/ODR → soft_reset。
 * 失败返回步骤号；全部成功返回 0。结果打到 COM4，并写入 g_selftest_rc。
 */
static int32_t sc7a20_driver_selftest(void) {
  ffl_sc7a20_raw_t raw;
  ffl_sc7a20_g_t g;
  uint8_t who = 0;
  int rc;
  int step = 1;

  rc = ffl_sc7a20_who_am_i(&s_accel, &who);
  printf("[sc7a20] who_am_i rc=%d val=0x%02x (expect 0x11)\n", rc, who);
  if (rc != 0 || who != 0x11u) {
    return step;
  }
  g_who_am_i = who;
  step++;

  rc = ffl_sc7a20_read_raw(&s_accel, &raw);
  printf("[sc7a20] read_raw rc=%d x=%d y=%d z=%d\n", rc, raw.x, raw.y, raw.z);
  if (rc != 0) {
    return step;
  }
  g_last_x = raw.x;
  g_last_y = raw.y;
  g_last_z = raw.z;
  step++;

  rc = ffl_sc7a20_read_g(&s_accel, &g);
  printf("[sc7a20] read_g rc=%d x_mg=%ld y_mg=%ld z_mg=%ld\n", rc,
         (long)(g.x * 1000.0f), (long)(g.y * 1000.0f), (long)(g.z * 1000.0f));
  if (rc != 0) {
    return step;
  }
  g_last_mg_x = (int32_t)(g.x * 1000.0f);
  g_last_mg_y = (int32_t)(g.y * 1000.0f);
  g_last_mg_z = (int32_t)(g.z * 1000.0f);
  step++;

  rc = ffl_sc7a20_set_range(&s_accel, FFL_SC7A20_RANGE_4G);
  printf("[sc7a20] set_range 4G rc=%d\n", rc);
  if (rc != 0) {
    return step;
  }
  step++;

  rc = ffl_sc7a20_read_raw(&s_accel, &raw);
  printf("[sc7a20] read_raw@4G rc=%d x=%d y=%d z=%d\n", rc, raw.x, raw.y,
         raw.z);
  if (rc != 0) {
    return step;
  }
  step++;

  rc = ffl_sc7a20_set_odr(&s_accel, FFL_SC7A20_ODR_50HZ);
  printf("[sc7a20] set_odr 50Hz rc=%d\n", rc);
  if (rc != 0) {
    return step;
  }
  step++;

  rc = ffl_sc7a20_set_axis_enable(&s_accel, true, true, true);
  printf("[sc7a20] set_axis_enable xyz rc=%d\n", rc);
  if (rc != 0) {
    return step;
  }
  step++;

  /* 恢复后续采样用的配置 */
  rc = ffl_sc7a20_set_range(&s_accel, FFL_SC7A20_RANGE_2G);
  if (rc != 0) {
    return step;
  }
  step++;
  rc = ffl_sc7a20_set_odr(&s_accel, FFL_SC7A20_ODR_100HZ);
  if (rc != 0) {
    return step;
  }
  step++;

  /* soft_reset 后需重新 init，验证复位路径 */
  rc = ffl_sc7a20_soft_reset(&s_accel);
  printf("[sc7a20] soft_reset rc=%d\n", rc);
  if (rc != 0) {
    return step;
  }
  step++;
  {
    ffl_sc7a20_config_t cfg;
    ffl_sc7a20_config_init(&cfg);
    cfg.range = FFL_SC7A20_RANGE_2G;
    cfg.odr = FFL_SC7A20_ODR_100HZ;
    rc = ffl_sc7a20_init(&s_accel, &cfg);
    printf("[sc7a20] re-init after reset rc=%d\n", rc);
    if (rc != 0) {
      return step;
    }
  }

  printf("[sc7a20] driver selftest PASS\n");
  return 0;
}

/** 把 raw 三轴打包成 6 字节小端记录，便于 ringbuffer 传输与事后解析。 */
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

/**
 * 一次采样：有传感器则 read_raw；否则写 0xFF 占位（Task 4 降级）。
 * 无论成败都 ringbuffer_put，证明缓冲链路与芯片无关。
 */
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
    g_last_x = raw.x;
    g_last_y = raw.y;
    g_last_z = raw.z;
    g_sample_ok_count++;
  } else {
    (void)memset(rec, 0xFF, sizeof(rec));
    g_sample_err_count++;
  }

  /* 环形缓冲总是写入，演示 put；满时自动丢弃尾部超出部分 */
  (void)ffl_ringbuffer_put(&s_rb, rec, sizeof(rec));
}

/** sw_timer 到期（主循环 process 上下文）：给 OSAL 任务投递采样事件。 */
static void sw_sample_expired(void *arg) {
  (void)arg;
  (void)osal_set_event(s_task_id, APP_EVT_SAMPLE);
}

/** 蜂鸣脉宽结束：关断 PB6（有源蜂鸣器，低=响）。 */
static void sw_beep_off(void *arg) {
  (void)arg;
  board_buzzer_off();
}

/** 短鸣：拉低 PB6，并用单次 sw_timer 在 BEEP_MS 后关断（period=0）。 */
static void beep_chirp(void) {
  board_buzzer_on();
  (void)ffl_sw_timer_start(&s_beep_timer, BEEP_MS, 0u, sw_beep_off, NULL);
}

/**
 * OSAL 任务 init：由 osal_Task_init() 回调一次。
 * 顺序：ringbuffer → 注入 I2C 并 bind sc7a20 → 配置/init → 启动定时器。
 */
static void app_task_init(uint8_t task_id) {
  ffl_sc7a20_config_t config;
  int rc;

  s_task_id = task_id;

  ffl_ringbuffer_init(&s_rb, s_rb_pool, sizeof(s_rb_pool));

  /* ---- Task 3：ports/stm32/f1 通用 I2C 南向 ----
   * board 负责 I2C1 实例/引脚；port 只做 ffl_transport 翻译。
   * 无 port 时：自写 ops + transport 再 bind（见 docs/tasks/03）。
   */
  {
    static ffl_stm32f1_i2c_ctx_t i2c_ctx;
    static ffl_transport_t i2c_tr;

    i2c_ctx.hi2c = board_i2c1_handle();
    i2c_ctx.timeout_ms = 50u;
    rc = ffl_stm32f1_i2c_transport_setup(&i2c_tr, &i2c_ctx,
                                         FFL_SC7A20_DEFAULT_ADDR7_L);
    if (rc == 0) {
      rc = ffl_stm32f1_sc7a20_bind(&s_accel, &i2c_tr,
                                   FFL_SC7A20_DEFAULT_ADDR7_L);
    }
  }
  if (rc == 0) {
    ffl_sc7a20_config_init(&config);
    config.range = FFL_SC7A20_RANGE_2G;
    config.odr = FFL_SC7A20_ODR_100HZ;
    rc = ffl_sc7a20_init(&s_accel, &config);
  }
  s_sensor_ok = (uint8_t)((rc == 0) ? 1u : 0u);

  printf("[boot] stm32-lora component test, sensor_ok=%u\n",
         (unsigned)s_sensor_ok);
  if (s_sensor_ok != 0u) {
    g_selftest_rc = sc7a20_driver_selftest();
  } else {
    g_selftest_rc = -1;
    printf("[sc7a20] skip selftest (bind/init failed)\n");
  }
  beep_chirp(); /* 上电自检鸣叫 */

  /* ---- ffl.osal：任务级 reload 定时器 ---- */
  (void)osal_start_reload_timer(task_id, APP_EVT_LED, LED_HEARTBEAT_MS);
  (void)osal_start_reload_timer(task_id, APP_EVT_STATS, STATS_PERIOD_MS);

  /* ---- ffl.sw_timer：周期触发采样 ---- */
  (void)ffl_sw_timer_start(&s_sample_timer, SAMPLE_PERIOD_MS, SAMPLE_PERIOD_MS,
                           sw_sample_expired, NULL);
}

/**
 * OSAL 任务事件处理：返回未处理完的事件位（本示例全部清掉）。
 * 必须在任务上下文运行，可安全调用组件与 printf。
 */
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
    /* UART1 -> COM4：无 SC7A20 时 err 递增、ok 停住，即可证明调度链路 */
    printf("[stats] sensor_ok=%u ok=%lu err=%lu drained=%lu raw=(%d,%d,%d)\n",
           (unsigned)s_sensor_ok, (unsigned long)g_sample_ok_count,
           (unsigned long)g_sample_err_count, (unsigned long)g_drained_bytes,
           (int)g_last_x, (int)g_last_y, (int)g_last_z);
    beep_chirp(); /* 每 3s 与统计同步短鸣 */
    events &= (uint16_t)~APP_EVT_STATS;
  }

  return events;
}

/**
 * 注册到 ffl.osal：init 回调 + 事件回调 + 任务栈/优先级参数（按 osal API）。
 * 必须在 main 里 osal_init_system() 之后、osal_Task_init() 之前调用。
 */
void app_task_register(void) {
  osal_add_Task(app_task_init, app_task_event, 1u);
}
