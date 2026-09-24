# Task 1：纯软件组件（无需 port）

**组件：** `ffl.ringbuffer`、`ffl.sw_timer`  
**仓库路径：** `components/foundation/`

## 为什么它们不用 port

这两个组件只依赖：

- 一块静态内存（ringbuffer 池）
- 毫秒 tick 与临界区（sw_timer 通过钩子注入，可选）

**没有** I2C/SPI/GPIO，因此**不需要** `ffl.driver_port`，也不需要 `ports/`。

## 接入步骤

1. 在 `xmake.lua` 里 `add_files` 组件 `src/*.c` 与 `include/`。
2. 应用里直接 `#include "ffl/ringbuffer.h"` / `"ffl/sw_timer.h"`。
3. 初始化后即可使用。

```c
/* app_task.c 里的用法摘要 */

/* 1) 环形缓冲：固定池 + put/get */
ffl_ringbuffer_init(&s_rb, s_rb_pool, sizeof(s_rb_pool));
ffl_ringbuffer_put(&s_rb, rec, 6);   /* 采样写入 */
ffl_ringbuffer_get(&s_rb, drain, n); /* 统计时成块取出 */

/* 2) 软件定时器：先给锁钩子（裸机上可与 OSAL 共用），再 init 轮 */
ffl_sw_timer_set_lock_hooks(bsp_critical_enter, bsp_critical_exit);
ffl_sw_timer_wheel_init(1u); /* 1ms 一格 */

/* 3) 周期定时：500ms 触发一次采样事件 */
ffl_sw_timer_start(&s_sample_timer, 500u, 500u, sw_sample_expired, NULL);
```

SysTick 里推进时间轮：

```c
/* bsp/src/stm32f1xx_it.c */
void SysTick_Handler(void) {
  HAL_IncTick();
  osal_update_timers();      /* Task 2 */
  ffl_sw_timer_tick_isr();   /* Task 1：ISR 只推轮 */
}
```

到期回调在**主循环**执行（不要在 ISR 里做业务）：

```c
/* main.c */
ffl_sw_timer_process();
```

## 本固件中的现象

| 行为 | 代码 |
|------|------|
| 每 500ms 采样一次 | `SAMPLE_PERIOD_MS` + `sw_sample_expired` |
| 每条记录 6 字节 | `SAMPLE_REC_BYTES`（x/y/z int16 LE） |
| 3s 成块清空 | `APP_EVT_STATS` 里 `ffl_ringbuffer_get` |

COM4：`drained=` 每次增加约 `采样次数 × 6`。

## 练习

- 把 `SAMPLE_PERIOD_MS` 改为 200，观察 `ok` 增速。
- 缩小 `RB_CAP_SAMPLES`，观察满时 put 行为（组件文档）。

下一篇：[02-runtime-osal.md](02-runtime-osal.md)
