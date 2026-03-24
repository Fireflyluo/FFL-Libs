# utils 工具模块

`utils/` 目录提供与平台解耦的基础组件，当前包含：

- `ringbuffer`：通用环形缓冲区
- `sw_timer`：时间轮软件定时器

## 构建方式（xmake）

在仓库根目录执行：

```bash
xmake f --utils_ringbuffer=y --utils_sw_timer=y
xmake
```

可选开关：

- `--utils_ringbuffer=y/n`：启用或禁用 ringbuffer
- `--utils_sw_timer=y/n`：启用或禁用 sw_timer

## sw_timer 适配说明

`sw_timer` 已移除对 `ch32v20x.h` 的硬依赖，改为通过钩子函数接入平台临界区。

在中断或多线程环境下，建议在初始化时注册锁钩子：

```c
#include "sw_timer.h"

extern void __disable_irq(void);
extern void __enable_irq(void);

void app_timer_init(void)
{
    sw_timer_set_lock_hooks(__disable_irq, __enable_irq);
    sw_timer_wheel_init(1); /* 1ms tick */
}



/**
 * @brief sw_timer_tick_isr 定时器tick中断服务函数
 * 
 * @note 此函数应在系统定时器中断中调用，通常每tick_ms毫秒调用一次
 *       负责推进时间轮指针并检查到期的定时器
 */


void TIM2_IRQHandler(void)
{
    sw_timer_tick_isr();
}
```

如果是单线程且无中断竞争场景，也可以不注册钩子。
