# SC7A20HTR 加速度计驱动说明文档

## 1. 概述

SC7A20HTR是一款高性能三轴数字加速度计，支持±2g/±4g/±8g/±16g量程范围，最高输出数据率可达4.434kHz。本驱动提供了完整的同步和异步操作接口，支持多种工作模式和高级功能。

### 1.1 主要特性

- **多量程支持**：±2g, ±4g, ±8g, ±16g
- **高数据率**：最高4.434kHz输出频率
- **多种工作模式**：正常模式、低功耗模式、高分辨率模式、电源关断模式
- **灵活的通信接口**：I2C/SPI（本驱动主要支持I2C）
- **同步/异步操作**：支持阻塞式同步操作和非阻塞式异步操作
- **丰富的配置选项**：轴使能控制、数据滤波、中断配置等
- **TMOS RTOS友好**：提供专门的TMOS任务集成示例

### 1.2 驱动架构

```
SC7A20HTR Driver
├── Core Layer (sc7a20_core.c/h)      # 核心驱动实现
├── HAL Layer (sc7a20_hal.h)          # 硬件抽象层接口
├── Async Layer (sc7a20_async.h/c)    # 异步操作支持
├── Platform Layer (platform/)         # 平台特定实现
└── Examples (example/)               # 使用示例
```

## 2. 配置选项

### 2.1 编译时配置

| 宏定义 | 默认值 | 说明 |
|--------|--------|------|
| `SC7A20_ASYNC_SUPPORT` | 0 | 启用异步操作支持 |
| `SC7A20_INTERNAL_BUFFER_SIZE` | 6 | 内部缓冲区大小（字节） |
| `SC7A20_ASYNC_DEFAULT_TIMEOUT_MS` | 100 | 异步操作默认超时时间（毫秒） |

### 2.2 设备配置结构

```c
typedef struct {
    uint8_t i2c_addr;          // I2C地址 (0x18 或 0x19)
    sc7a20_accel_fs_t range;   // 量程范围
    sc7a20_accel_odr_t odr;    // 输出数据率
    uint8_t enable_axis[3];    // 三轴使能配置 [X, Y, Z]
    bool block_data_update;    // 块数据更新使能
    bool high_resolution_mode; // 高分辨率模式
    bool low_power_mode;       // 低功耗模式
} sc7a20_config_t;
```

### 2.3 量程和数据率枚举

**量程选项 (`sc7a20_accel_fs_t`)**:
- `SC7A20_ACCEL_FS_2G`  : ±2g
- `SC7A20_ACCEL_FS_4G`  : ±4g  
- `SC7A20_ACCEL_FS_8G`  : ±8g
- `SC7A20_ACCEL_FS_16G` : ±16g

**数据率选项 (`sc7a20_accel_odr_t`)**:
- `SC7A20_ACCEL_ODR_POWER_DOWN` : 电源关断模式
- `SC7A20_ACCEL_ODR_1_56HZ`     : 1.56Hz
- `SC7A20_ACCEL_ODR_12_5HZ`     : 12.5Hz
- `SC7A20_ACCEL_ODR_25HZ`       : 25Hz
- `SC7A20_ACCEL_ODR_50HZ`       : 50Hz
- `SC7A20_ACCEL_ODR_100HZ`      : 100Hz
- `SC7A20_ACCEL_ODR_200HZ`      : 200Hz
- `SC7A20_ACCEL_ODR_400HZ`      : 400Hz
- `SC7A20_ACCEL_ODR_800HZ`      : 800Hz
- `SC7A20_ACCEL_ODR_1_48KHZ`    : 1.48kHz
- `SC7A20_ACCEL_ODR_2_66KHZ`    : 2.66kHz
- `SC7A20_ACCEL_ODR_4_434KHZ`   : 4.434kHz

## 3. API 接口

### 3.1 设备管理接口

```c
// 初始化传感器
sc7a20_status_t sc7a20_init(sc7a20_handle_t handle, const sc7a20_ops_t *ops, const sc7a20_config_t *config);

// 反初始化传感器
sc7a20_status_t sc7a20_deinit(sc7a20_handle_t handle);

// 软件复位
sc7a20_status_t sc7a20_reset(sc7a20_handle_t handle);

// 快速初始化（使用默认配置）
sc7a20_status_t sc7a20_quick_init(sc7a20_handle_t handle, const sc7a20_ops_t *ops, uint8_t i2c_addr);
```

### 3.2 数据采集接口

```c
// 读取加速度数据（包含原始数据和g值）
sc7a20_status_t sc7a20_read_acceleration(sc7a20_handle_t handle, sc7a20_accel_data_t *data);

// 读取原始加速度数据
sc7a20_status_t sc7a20_read_raw_data(sc7a20_handle_t handle, int16_t *x, int16_t *y, int16_t *z);

// 检查新数据是否就绪
sc7a20_status_t sc7a20_is_data_ready(sc7a20_handle_t handle, bool *ready);

// 从实时数据寄存器读取数据
sc7a20_status_t sc7a20_read_new_data(sc7a20_handle_t handle, sc7a20_accel_data_t *data);
```

### 3.3 配置管理接口

```c
// 设置加速度计量程
sc7a20_status_t sc7a20_set_range(sc7a20_handle_t handle, sc7a20_accel_fs_t range);

// 设置输出数据率
sc7a20_status_t sc7a20_set_output_data_rate(sc7a20_handle_t handle, sc7a20_accel_odr_t odr);

// 设置轴使能状态
sc7a20_status_t sc7a20_set_axis_enable(sc7a20_handle_t handle, bool x_enable, bool y_enable, bool z_enable);

// 获取当前配置
sc7a20_status_t sc7a20_get_config(sc7a20_handle_t handle, sc7a20_config_t *config);
```

### 3.4 电源管理接口

```c
// 进入睡眠模式
sc7a20_status_t sc7a20_enter_sleep(sc7a20_handle_t handle);

// 从睡眠模式唤醒
sc7a20_status_t sc7a20_wakeup(sc7a20_handle_t handle);

// 设置电源模式
sc7a20_status_t sc7a20_set_power_mode(sc7a20_handle_t handle, sc7a20_power_mode_t mode);
```

### 3.5 异步接口（需要启用 `SC7A20_ASYNC_SUPPORT`）

```c
// 异步写寄存器
sc7a20_status_t sc7a20_write_register_async(sc7a20_handle_t handle, 
                                           uint8_t reg, 
                                           const uint8_t *data, 
                                           uint16_t len,
                                           void (*callback)(sc7a20_handle_t handle, sc7a20_status_t status));

// 异步读寄存器
sc7a20_status_t sc7a20_read_register_async(sc7a20_handle_t handle, 
                                          uint8_t reg, 
                                          uint8_t *data, 
                                          uint16_t len,
                                          void (*callback)(sc7a20_handle_t handle, sc7a20_status_t status));

// 异步读取加速度数据
sc7a20_status_t sc7a20_read_acceleration_async(sc7a20_handle_t handle,
                                              sc7a20_accel_data_t *data,
                                              void (*callback)(sc7a20_handle_t handle, sc7a20_status_t status));
```

## 4. 使用示例

### 4.1 同步模式基本使用

```c
#include "platform.h"

int main(void)
{
    // 初始化加速度计（同步模式）
    if (accel_init() != 0) {
        return -1;
    }
    
    while (1) {
        read_acceleration_data(); // 读取并打印加速度数据
        Delay_Ms(100); // 100ms间隔
    }
    
    return 0;
}
```

### 4.2 异步模式使用（TMOS任务）

```c
#include "platform.h"
#include "tmos_task.h"

#if SC7A20_ASYNC_SUPPORT

// 全局变量
static sc7a20_accel_data_t g_accel_data;
static bool g_accel_data_ready = false;
static uint8_t accel_task_id = 0;

// 异步读取完成回调
void accel_read_callback(sc7a20_handle_t handle, sc7a20_status_t status)
{
    if (status == SC7A20_OK) {
        g_accel_data_ready = true;
        tmos_set_event(accel_task_id, ACCEL_DATA_READY_EVENT);
    }
}

// TMOS任务函数
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

// 应用初始化
int app_init(void)
{
    return accel_init_async();
}

#endif
```

### 4.3 异步模式使用（主循环）

```c
#include "platform.h"

#if SC7A20_ASYNC_SUPPORT

static sc7a20_accel_data_t g_accel_data;
static volatile bool g_accel_data_ready = false;

void accel_read_callback(sc7a20_handle_t handle, sc7a20_status_t status)
{
    if (status == SC7A20_OK) {
        g_accel_data_ready = true;
    }
}

int main(void)
{
    if (accel_init_async() != 0) {
        return -1;
    }
    
    while (1) {
        sc7a20_handle_t handle = get_accel_handle();
        if (handle->async_ctx.state == SC7A20_ASYNC_IDLE) {
            sc7a20_read_acceleration_async(handle, &g_accel_data, accel_read_callback);
        }
        
        if (g_accel_data_ready) {
            printf("X=%fg, Y=%fg, Z=%fg\n", 
                   g_accel_data.x_g, g_accel_data.y_g, g_accel_data.z_g);
            g_accel_data_ready = false;
        }
        
        delay_ms(10);
    }
    
    return 0;
}

#endif
```

## 5. 硬件抽象层（HAL）

### 5.1 同步操作接口

```c
typedef struct {
    sc7a20_hal_write_fn write;    // 写寄存器函数
    sc7a20_hal_read_fn read;      // 读寄存器函数  
    sc7a20_hal_delay_fn delay_ms; // 延时函数
    void *user_data;              // 用户数据
} sc7a20_ops_t;
```

### 5.2 异步操作接口（启用异步支持时）

```c
typedef struct {
    // ... 同步接口 ...
    sc7a20_hal_write_async_fn write_async; // 异步写寄存器函数
    sc7a20_hal_read_async_fn read_async;   // 异步读寄存器函数
    sc7a20_hal_check_busy_fn is_busy;      // 检查硬件是否忙
} sc7a20_ops_t;
```

### 5.3 CH32V208平台实现

驱动已为CH32V208平台提供了完整的平台层实现，位于 `platform/platform.c` 文件中，包括：

- I2C同步读写实现
- I2C异步读写实现（需要I2C中断支持）
- 设备句柄管理
- 回调函数处理

## 6. 状态码定义

| 状态码 | 说明 |
|--------|------|
| `SC7A20_OK` | 操作成功 |
| `SC7A20_ERROR` | 一般错误 |
| `SC7A20_TIMEOUT` | 超时错误 |
| `SC7A20_INVALID_PARAM` | 无效参数 |
| `SC7A20_NOT_INIT` | 未初始化 |
| `SC7A20_COMM_ERROR` | 通信错误 |
| `SC7A20_DEVICE_NOT_FOUND` | 设备未找到 |

## 7. 注意事项

### 7.1 内存使用
- 驱动采用静态内存分配，用户需要预先分配 `sc7a20_dev_t` 结构体
- 异步模式会增加每个设备结构体约32字节的内存占用
- 内部缓冲区大小为6字节（用于加速度数据）

### 7.2 线程安全
- 在多任务环境中，确保同一设备句柄不会被多个任务同时访问
- 可以使用互斥锁或信号量来保护设备句柄
- TMOS环境下建议使用事件驱动的方式

### 7.3 错误处理
- 始终检查API返回的状态码
- 在异步回调函数中处理操作状态
- 实现超时检测机制防止死锁

### 7.4 性能考虑
- 异步模式减少了CPU阻塞时间，提高了系统响应性
- 但在高频率读取场景下，同步模式可能更简单高效
- 选择合适的数据率以平衡性能和功耗

## 8. 兼容性说明

- 当 `SC7A20_ASYNC_SUPPORT=0` 时，所有异步代码都不会编译，不影响现有同步代码
- 同步接口始终可用，与异步支持宏无关
- 可以在同一项目中混合使用同步和异步接口（但不推荐对同一设备实例混用）

## 9. 相关文件

- **核心驱动**: `sc7a20_core.c/h`
- **异步支持**: `sc7a20_core_async.c`, `sc7a20_async.h`
- **硬件抽象**: `sc7a20_hal.h`
- **寄存器定义**: `sc7a20_reg.h`
- **基础定义**: `sc7a20_def.h`
- **平台实现**: `platform/platform.c/h`
- **使用示例**: `example/` 目录
- **详细指南**: `async_usage_guide.md`

## 10. 版本信息

- **驱动版本**: 1.0
- **支持芯片**: SC7A20HTR
- **测试平台**: CH32V208 ：裸机、 TMOS 
- **最后更新**: 2026-02-28