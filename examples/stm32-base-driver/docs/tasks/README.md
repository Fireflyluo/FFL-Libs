# 任务列表：仓库用法

本目录把 `stm32-base-driver` 拆成可单独理解的 **接入任务**。  
每篇只讲一种仓库用法，代码都落在同一份固件里。

| 文档 | 你学会什么 |
|------|------------|
| [01-foundation.md](01-foundation.md) | 无 port 的纯软件组件怎么嵌进应用 |
| [02-runtime-osal.md](02-runtime-osal.md) | OSAL 如何靠钩子挂在裸机 SysTick 上 |
| [03-driver-with-port.md](03-driver-with-port.md) | 驱动 + **ports/**：已有南向直接接 vs 无 port 自实现 |
| [04-driver-no-hw.md](04-driver-no-hw.md) | 没有传感器/port 时如何降级仍验证链路 |
| [05-features.md](05-features.md) | ringbuffer / sw_timer / atomic 功能点速查 |

返回总览：[../readme.md](../readme.md)
