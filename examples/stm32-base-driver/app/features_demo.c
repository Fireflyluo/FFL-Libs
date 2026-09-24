/**
 * @file features_demo.c
 * @brief Task 5：纯软件组件功能自检（不需要任何 MCU port）。
 *
 * 目的：在不依赖 I2C/传感器的前提下，证明 foundation 组件 API 可用。
 * 与 app_task.c 的业务链路并行、互不影响；上电打印 [feat] ...。
 *
 * 组件来源（均在 components/，本文件不复制实现）：
 *   ffl.atomic      components/foundation/atomic/include/ffl_atomic.h
 *   ffl.ringbuffer  components/foundation/ringbuffer/
 *   ffl.sw_timer    components/foundation/sw-timer/  （单次定时需主循环 process）
 *
 * 文档：docs/tasks/05-features.md
 */
#include "ffl_atomic.h"

#include <stdio.h>
#include <string.h>

#include "board.h"
#include "ffl/ringbuffer.h"
#include "ffl/sw_timer.h"

static ffl_sw_timer_t s_feat_oneshot;
static volatile uint8_t s_feat_lock;
static volatile uint8_t s_feat_flag;
static volatile uint32_t s_feat_oneshot_fired;

static void feat_oneshot_cb(void *arg) {
  (void)arg;
  s_feat_oneshot_fired++;
}

static int feat_atomic_demo(void) {
  int ok = 1;
  uint8_t v = 0;

  if (!ffl_atomic_try_lock_u8(&s_feat_lock)) {
    ok = 0;
  }
  /* 已上锁，再抢应失败 */
  if (ffl_atomic_try_lock_u8(&s_feat_lock)) {
    ok = 0;
  }
  ffl_atomic_unlock_u8(&s_feat_lock);
  if (!ffl_atomic_try_lock_u8(&s_feat_lock)) {
    ok = 0;
  }
  ffl_atomic_unlock_u8(&s_feat_lock);

  (void)ffl_atomic_fetch_add_u8(&v, 3u);
  if (v != 3u) {
    ok = 0;
  }
  if (!ffl_atomic_compare_exchange_u8(&v, 3u, 10u)) {
    ok = 0;
  }
  if (ffl_atomic_load_u8(&s_feat_flag) != 0u) {
    ok = 0;
  }
  ffl_atomic_store_u8(&s_feat_flag, 1u);
  if (ffl_atomic_load_u8(&s_feat_flag) != 1u) {
    ok = 0;
  }

  printf("[feat] atomic %s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : -1;
}

static int feat_ringbuffer_demo(void) {
  uint8_t pool[16];
  uint8_t in[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  uint8_t out[16];
  ffl_ringbuffer_t rb;
  size_t n;
  ffl_ringbuffer_state_t st;
  int ok = 1;

  ffl_ringbuffer_init(&rb, pool, sizeof(pool));
  st = ffl_ringbuffer_status(&rb);
  if (st != FFL_RINGBUFFER_EMPTY) {
    ok = 0;
  }

  n = ffl_ringbuffer_put(&rb, in, 8u);
  if (n != 8u) {
    ok = 0;
  }
  n = ffl_ringbuffer_put(&rb, in, 8u);
  if (n != 8u) { /* 池 16B，应写满 */
    ok = 0;
  }
  st = ffl_ringbuffer_status(&rb);
  if (st != FFL_RINGBUFFER_FULL) {
    ok = 0;
  }

  /* 满后再 put：空间不足时丢弃超出输入，返回实际写入（应为 0） */
  n = ffl_ringbuffer_put(&rb, in, 4u);
  if (n != 0u) {
    ok = 0;
  }

  memset(out, 0, sizeof(out));
  n = ffl_ringbuffer_get(&rb, out, 8u);
  if (n != 8u || out[0] != 1u || out[7] != 8u) {
    ok = 0;
  }

  printf("[feat] ringbuffer %s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : -1;
}

static int feat_sw_timer_oneshot(void) {
  s_feat_oneshot_fired = 0;
  if (ffl_sw_timer_start(&s_feat_oneshot, 30u, 0u, feat_oneshot_cb, NULL) != 0) {
    printf("[feat] sw_timer FAIL start\n");
    return -1;
  }
  /* 单次到期需主循环 ffl_sw_timer_process()，上电阶段只验证 start=0 */
  printf("[feat] sw_timer one-shot armed (period=0)\n");
  return 0;
}

void features_demo_run(void) {
  int fails = 0;

  fails += feat_atomic_demo();
  fails += feat_ringbuffer_demo();
  fails += feat_sw_timer_oneshot();

  printf("[feat] summary fails=%d (see docs/tasks/05-features.md)\n", fails);
}
