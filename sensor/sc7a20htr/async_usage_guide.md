# SC7A20HTR 异步功能使用指南

本指南面向 `SC7A20_ASYNC_SUPPORT` 异步接口，适用于 TMOS 任务或主循环方式。

## 1. 启用异步功能

方式一：在配置头文件中开启

```c
// config.h
#ifndef CONFIG_H
#define CONFIG_H

#define SC7A20_ASYNC_SUPPORT 1

#endif
```

方式二：编译器宏定义

```bash
gcc -DSC7A20_ASYNC_SUPPORT=1 ...
```

## 2. 平台层准备

异步模式需要底层 I2C 提供异步读写与回调通知。CH32V208 平台已在 `platform/platform.c` 实现了以下对接函数：

- `i2c_write_complete_callback`
- `i2c_read_complete_callback`
- `i2c_error_callback_impl`

应用层需要在 I2C 中断回调中转发到这些函数，示例：

```c
// I2C TX 完成回调
void i2c_master_tx_cplt_callback(i2c_num_t i2c_num)
{
    extern void i2c_write_complete_callback(i2c_num_t i2c_num);
    i2c_write_complete_callback(i2c_num);
}

// I2C RX 完成回调
void i2c_master_rx_cplt_callback(i2c_num_t i2c_num)
{
    extern void i2c_read_complete_callback(i2c_num_t i2c_num);
    i2c_read_complete_callback(i2c_num);
}

// I2C 错误回调
void i2c_error_callback(i2c_num_t i2c_num, i2c_ErrCode_t errcode)
{
    extern void i2c_error_callback_impl(i2c_num_t i2c_num, i2c_ErrCode_t errcode);
    i2c_error_callback_impl(i2c_num, errcode);
}
```

## 3. 初始化

优先使用平台封装接口：

```c
#include "platform.h"

int main(void)
{
    if (accel_init_async() != 0) {
        return -1;
    }

    return 0;
}
```

如需自行适配，请使用 `sc7a20_init`，并在 `sc7a20_ops_t` 中填充 `write_async`、`read_async`、`is_busy`。

## 4. 主循环示例

主循环模式下建议在空闲状态触发异步读取：

```c
#include "platform.h"

extern sc7a20_handle_t get_accel_handle(void); // 若未在头文件声明，请在应用层自行声明

static sc7a20_accel_data_t g_accel_data;
static volatile bool g_data_ready = false;

void accel_callback(sc7a20_handle_t handle, sc7a20_status_t status)
{
    if (status == SC7A20_OK) {
        g_data_ready = true;
    }
}

int main(void)
{
    accel_init_async();

    while (1) {
        sc7a20_handle_t handle = get_accel_handle();
        if (handle->async_ctx.state == SC7A20_ASYNC_IDLE) {
            sc7a20_read_acceleration_async(handle, &g_accel_data, accel_callback);
        }

        if (g_data_ready) {
            printf("X=%fg, Y=%fg, Z=%fg\n",
                   g_accel_data.x_g, g_accel_data.y_g, g_accel_data.z_g);
            g_data_ready = false;
        }

        Delay_Ms(10);
    }
}
```

## 5. TMOS 任务示例

```c
#include "platform.h"
#include "tmos_task.h"

extern sc7a20_handle_t get_accel_handle(void);

static sc7a20_accel_data_t g_accel_data;
static bool g_accel_data_ready = false;
static uint8_t accel_task_id = 0;

void accel_read_callback(sc7a20_handle_t handle, sc7a20_status_t status)
{
    if (status == SC7A20_OK) {
        g_accel_data_ready = true;
        tmos_set_event(accel_task_id, ACCEL_DATA_READY_EVENT);
    }
}

void accel_task_process(uint8_t task_id, uint16_t events)
{
    accel_task_id = task_id;

    if (events & ACCEL_READ_TRIGGER_EVENT) {
        sc7a20_handle_t handle = get_accel_handle();
        sc7a20_read_acceleration_async(handle, &g_accel_data, accel_read_callback);
    }

    if (events & ACCEL_DATA_READY_EVENT) {
        if (g_accel_data_ready) {
            printf("X=%fg, Y=%fg, Z=%fg\n",
                   g_accel_data.x_g, g_accel_data.y_g, g_accel_data.z_g);
            g_accel_data_ready = false;
            tmos_set_event(accel_task_id, ACCEL_READ_TRIGGER_EVENT);
        }
    }
}
```

## 6. 超时与状态管理

- 默认超时宏：`SC7A20_ASYNC_DEFAULT_TIMEOUT_MS`。
- `sc7a20_check_async_timeout` 目前为占位逻辑，若需要超时控制建议在应用层结合系统时钟实现。
- 异步接口不支持并发，确保 `handle->async_ctx.state == SC7A20_ASYNC_IDLE` 后再发起新操作。

## 7. 常见问题与建议

- 回调运行在中断上下文时，请避免执行阻塞或耗时操作。
- 若 I2C 仅支持同步模式，请关闭 `SC7A20_ASYNC_SUPPORT`，改用同步接口。
- 若需要自定义超时或重试策略，请在应用层封装读取流程。
