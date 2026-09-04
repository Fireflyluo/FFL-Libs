# SC7A20HTR 加速度计驱动说明

## 1. 概述

SC7A20HTR 是一款三轴数字加速度计，支持 ±2g/±4g/±8g/±16g 量程，最高输出数据率可达 4.434kHz。本驱动提供同步与异步两套接口，默认使用 I2C 通信，面向 TMOS/裸机等场景。

## 2. 目录结构

- `sc7a20_core.c/h`：核心驱动实现与接口
- `sc7a20_core_async.c`：异步接口实现
- `inc/sc7a20_def.h`：基础类型、枚举与配置结构
- `inc/sc7a20_reg.h`：寄存器与位域定义
- `inc/sc7a20_hal.h`：硬件抽象层接口
- `inc/sc7a20_async.h`：异步辅助接口与宏定义
- `platform/`：平台适配层（当前提供 CH32V208 示例）
- `example/`：示例代码
- `async_usage_guide.md`：异步使用指南

## 3. 快速开始（同步）

使用平台封装接口：

```c
#include "sc7a20_ch32_adapter.h"

int main(void)
{
    if (accel_init() != 0) {
        return -1;
    }

    while (1) {
        read_acceleration_data();
        Delay_Ms(100);
    }

    return 0;
}
```

如需自行适配平台，请参考 `sc7a20_ops_t` 并调用 `sc7a20_init` 完成初始化。

## 4. 编译配置宏

| 宏 | 默认值 | 说明 |
| --- | --- | --- |
| `SC7A20_ASYNC_SUPPORT` | 0 | 启用异步接口支持（见 `inc/sc7a20_def.h`） |
| `SC7A20_ASYNC_DEFAULT_TIMEOUT_MS` | 100 | 异步默认超时（见 `inc/sc7a20_async.h`） |
| `SC7A20_INTERNAL_BUFFER_SIZE` | 6 | 异步内部缓冲区大小（见 `inc/sc7a20_async.h`） |

## 5. 主要 API 速览

设备管理：
- `sc7a20_init` / `sc7a20_deinit` / `sc7a20_reset`
- `sc7a20_is_ready` / `sc7a20_get_chip_id` / `sc7a20_get_status`

数据读取：
- `sc7a20_read_acceleration`
- `sc7a20_read_raw_data`
- `sc7a20_is_data_ready` / `sc7a20_read_new_data`

配置管理：
- `sc7a20_set_range`
- `sc7a20_set_output_data_rate`
- `sc7a20_set_axis_enable`
- `sc7a20_get_config`
- `sc7a20_config_high_performance` / `sc7a20_config_low_power`

电源管理：
- `sc7a20_enter_sleep` / `sc7a20_wakeup`
- `sc7a20_set_power_mode`

异步接口（`SC7A20_ASYNC_SUPPORT=1`）：
- `sc7a20_write_register_async` / `sc7a20_read_register_async`
- `sc7a20_read_acceleration_async` / `sc7a20_read_raw_data_async`
- `sc7a20_set_range_async` / `sc7a20_set_output_data_rate_async`
- `sc7a20_set_axis_enable_async` / `sc7a20_set_power_mode_async`

## 6. 平台适配说明

平台适配通过 `sc7a20_ops_t` 提供 I2C 读写与延时接口。异步模式需要底层 I2C 提供异步读写与忙状态查询，并在中断回调中通知驱动完成状态。当前 CH32V208 适配位于 `platform/sc7a20_ch32_adapter.c`。

## 7. 注意事项

- 驱动采用静态内存模型，需由用户分配 `sc7a20_dev_t` 实例。
- 同一设备句柄不要在多任务中并发访问，必要时使用互斥或事件同步。
- 异步接口由底层 I2C 回调驱动，回调中尽量保持轻量处理。
- `sc7a20_check_async_timeout` 为占位逻辑，若需超时控制请在应用层实现。

## 8. 版本信息

- 版本：1.0
- 芯片：SC7A20HTR
- 平台：CH32V208 + TMOS
- 最后更新：2026-03-24

## Quick Include

```c
#include "sc7a20_core.h"
```
