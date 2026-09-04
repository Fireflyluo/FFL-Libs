# Software Timer

`ffl.sw_timer` 是一个不依赖 MCU HAL、RTOS 或板级全局句柄的时间轮软件定时器。它采用源码组件方式提供，适合需要调整实现、控制资源占用或注入平台临界区的工程。

## 运行模型

- 在系统 tick 中断中周期调用 `sw_timer_tick_isr()`，推进时间轮并收集到期定时器。
- 在主循环或任务上下文调用 `sw_timer_process()`，执行到期回调。
- `sw_timer_wheel_init(tick_ms)` 的参数定义系统 tick 周期，所有毫秒延时向上取整到 tick。
- `delay_ms == 0` 的定时器进入待处理队列，不会在启动函数中直接执行回调。
- 定时器节点由调用方持有，节点在活动期间不能释放或重复用于其他定时器。

## 并发与平台适配

组件不绑定具体临界区实现。多线程或中断与任务共享定时器时，使用 `sw_timer_set_lock_hooks()` 注入进入/退出临界区函数；`sw_timer_tick_isr()` 本身运行在中断上下文，不额外调用钩子。

```c
#include "sw_timer.h"

void app_timer_init(void)
{
    sw_timer_set_lock_hooks(platform_lock, platform_unlock);
    sw_timer_wheel_init(1U);
}
```

## Xmake 使用

```lua
includes("components/foundation/sw-timer")

target("app")
    set_kind("binary")
    set_languages("cxx17")
    add_files("src/*.cpp")
    add_deps("ffl.sw_timer")
```

host 测试使用 C++ 编译并调用 C 实现：

```powershell
xmake test -P . ffl.sw_timer.test
```
