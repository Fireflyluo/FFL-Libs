# Task 2：运行时组件 ffl.osal（轻量 port）

**组件：** `ffl.osal`  
**仓库路径：** `components/runtime/osal/`

## 和纯软件组件的差别

OSAL 提供任务、事件、reload 定时器、消息等，但**不绑定 RTOS**。  
裸机接入时需要 MCU 南向（**`ports/<芯片>/osal`**）：

| 能力 | STM32F1 |
|------|---------|
| 临界区 | **`ports/stm32/f1/osal/`**（PRIMASK，可嵌套） |
| 时间推进 | SysTick → `osal_update_timers()`（应用 ISR） |
| 任务调度 | 主循环 `osal_process_once()` |

本固件：

```c
#include "ffl_port_stm32f1_osal.h"
ffl_stm32f1_osal_port_init();
ffl_sw_timer_set_lock_hooks(ffl_stm32f1_critical_enter,
                            ffl_stm32f1_critical_exit);
```

无 port 时：自行实现 `enter/exit` 钩子（关开中断）。

## 接入步骤

```c
/* main.c */
ffl_stm32f1_osal_port_init();
osal_init_system();
app_task_register();      /* osal_add_Task(...) */
osal_Task_init();         /* 调各任务的 init */

for (;;) {
  osal_process_once();    /* 跑事件 */
  ffl_sw_timer_process();
  board_idle();           /* WFI */
}
```

```c
/* app_task.c */
static uint16_t app_task_event(uint8_t task_id, uint16_t events) {
  if (events & APP_EVT_LED) { board_led_toggle(); ... }
  if (events & APP_EVT_SAMPLE) { read_one_sample(); ... }
  if (events & APP_EVT_STATS) { /* ringbuffer get + printf */ }
  return events;
}
```

```c
/* reload 定时器：周期置事件 */
osal_start_reload_timer(task_id, APP_EVT_LED, 500);
osal_start_reload_timer(task_id, APP_EVT_STATS, 3000);
```

```c
/* sw_timer 回调里安全地给任务置事件（任务上下文） */
static void sw_sample_expired(void *arg) {
  (void)arg;
  osal_set_event(s_task_id, APP_EVT_SAMPLE);
}
```

## 和 Task 1 的协作关系

```text
SysTick 1ms
  ├─ HAL_IncTick
  ├─ osal_update_timers     → LED/STATS reload 到期 → 置事件
  └─ ffl_sw_timer_tick_isr  → 采样定时到 → 回调里置 EVT_SAMPLE

主循环
  ├─ osal_process_once      → 执行事件
  └─ ffl_sw_timer_process   → 执行到期回调
```

## 现象

- LED 心跳 500ms 翻转  
- 每 3s 打印一行 `[stats]`  
- 即使传感器不在，调度仍在跑  

下一篇：[03-driver-with-port.md](03-driver-with-port.md)
