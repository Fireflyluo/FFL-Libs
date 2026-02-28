# SC7A20HTR 异步功能使用指南

## 1. 启用异步功能

要在项目中启用SC7A20HTR的异步功能，需要在编译前定义 `SC7A20_ASYNC_SUPPORT` 宏。

### 方法一：在源文件中定义
```c
#define SC7A20_ASYNC_SUPPORT 1
#include "sc7a20_core.h"
```

### 方法二：在项目配置文件中定义
创建 `config.h` 文件：
```c
// config.h
#ifndef CONFIG_H
#define CONFIG_H

#define SC7A20_ASYNC_SUPPORT 1

#endif
```

### 方法三：在编译器命令行中定义
```bash
gcc -DSC7A20_ASYNC_SUPPORT=1 ...
```

## 2. 初始化异步模式

### 2.1 使用平台层初始化函数
```c
#include "platform.h"

int main(void)
{
    // 初始化加速度计（异步模式）
    int ret = accel_init_async();
    if (ret != 0) {
        // 处理初始化失败
        return -1;
    }
    
    // 继续其他初始化...
    return 0;
}
```

### 2.2 手动初始化
```c
#include "sc7a20_core.h"
#include "drv_i2c.h"

// 全局设备结构体
static sc7a20_dev_t g_accel_dev;

// I2C回调函数
void i2c_write_complete_callback(i2c_num_t i2c_num) { /* ... */ }
void i2c_read_complete_callback(i2c_num_t i2c_num) { /* ... */ }
void i2c_error_callback_impl(i2c_num_t i2c_num, i2c_ErrCode_t errcode) { /* ... */ }

int init_accelerometer(void)
{
    // 配置I2C为中断模式
    i2c_config_t i2c_config = {
        .clock_speed = 400000,
        .duty_cycle = I2C_DutyCycle_2,
        .own_address = 0,
        .enable_ack = true,
        .is_7_bit_address = true
    };
    
    i2c_init(I2C_NUM_1, &i2c_config);
    
    // 设置I2C回调
    // 注意：这些是弱函数，需要在应用层实现
    
    sc7a20_ops_t ops = {
        .write = i2c_write_reg,
        .read = i2c_read_reg,
        .delay_ms = Delay_Ms,
        .write_async = i2c_write_reg_async,
        .read_async = i2c_read_reg_async,
        .is_busy = i2c_is_busy,
        .user_data = NULL
    };
    
    sc7a20_config_t config = {
        .i2c_addr = SC7A20_I2C_ADDR_H,
        .range = SC7A20_ACCEL_FS_2G,
        .odr = SC7A20_ACCEL_ODR_50HZ,
        .enable_axis = {1, 1, 1},
        .block_data_update = true,
        .high_resolution_mode = true,
        .low_power_mode = false
    };
    
    sc7a20_status_t status = sc7a20_init(&g_accel_dev, &ops, &config);
    return (status == SC7A20_OK) ? 0 : -1;
}
```

## 3. 在主循环中使用异步功能

### 3.1 简单轮询模式
```c
#include "platform.h"

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
    // 初始化
    accel_init_async();
    
    while (1) {
        // 触发异步读取（如果当前没有操作进行中）
        sc7a20_handle_t handle = get_accel_handle();
        if (handle->async_ctx.state == SC7A20_ASYNC_IDLE) {
            sc7a20_read_acceleration_async(handle, &g_accel_data, accel_callback);
        }
        
        // 检查数据是否就绪
        if (g_data_ready) {
            printf("X=%fg, Y=%fg, Z=%fg\n", 
                   g_accel_data.x_g, g_accel_data.y_g, g_accel_data.z_g);
            g_data_ready = false;
        }
        
        delay_ms(10); // 避免CPU占用过高
    }
}
```

### 3.2 带超时检测的轮询模式
```c
static uint32_t operation_start_time = 0;
static const uint32_t timeout_ms = 100;

void main_loop_with_timeout(void)
{
    while (1) {
        sc7a20_handle_t handle = get_accel_handle();
        
        // 检查超时
        if (handle->async_ctx.state != SC7A20_ASYNC_IDLE) {
            if ((get_millis() - operation_start_time) > timeout_ms) {
                printf("操作超时!\n");
                handle->async_ctx.state = SC7A20_ASYNC_IDLE;
            }
        }
        
        // 触发新操作
        if (handle->async_ctx.state == SC7A20_ASYNC_IDLE) {
            sc7a20_read_acceleration_async(handle, &g_accel_data, accel_callback);
            operation_start_time = get_millis();
        }
        
        // 处理完成的数据
        if (g_data_ready) {
            // 处理数据...
            g_data_ready = false;
        }
        
        delay_ms(1);
    }
}
```

## 4. 在中断服务程序中使用

由于异步操作本身就是在中断上下文中完成的，通常不需要在ISR中直接调用异步API。但可以在ISR中触发异步操作：

```c
// 定时器中断
void TIM2_IRQHandler(void)
{
    static bool trigger_read = true;
    
    if (trigger_read) {
        sc7a20_handle_t handle = get_accel_handle();
        if (handle->async_ctx.state == SC7A20_ASYNC_IDLE) {
            sc7a20_read_acceleration_async(handle, &g_accel_data, accel_callback);
            trigger_read = false;
        }
    }
    
    // 清除中断标志
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}
```

## 5. I2C回调函数实现

异步功能依赖于I2C中断回调。需要在应用层实现以下弱函数：

```c
// I2C写完成回调
void i2c_master_tx_cplt_callback(i2c_num_t i2c_num)
{
    // 调用平台层的写完成处理
    extern void i2c_write_complete_callback(i2c_num_t i2c_num);
    i2c_write_complete_callback(i2c_num);
}

// I2C读完成回调  
void i2c_master_rx_cplt_callback(i2c_num_t i2c_num)
{
    // 调用平台层的读完成处理
    extern void i2c_read_complete_callback(i2c_num_t i2c_num);
    i2c_read_complete_callback(i2c_num);
}

// I2C错误回调
void i2c_error_callback(i2c_num_t i2c_num, i2c_ErrCode_t errcode)
{
    // 调用平台层的错误处理
    extern void i2c_error_callback_impl(i2c_num_t i2c_num, i2c_ErrCode_t errcode);
    i2c_error_callback_impl(i2c_num, errcode);
}
```

## 6. 注意事项

### 6.1 内存使用
- 异步模式会增加每个 `sc7a20_dev_t` 结构体的内存占用（约32字节）
- 内部缓冲区大小为6字节（用于加速度数据）

### 6.2 线程安全
- 在多任务环境中，确保同一设备句柄不会被多个任务同时访问
- 可以使用互斥锁或信号量来保护设备句柄

### 6.3 错误处理
- 始终检查异步API的返回值
- 在回调函数中处理操作状态
- 实现超时检测机制防止死锁

### 6.4 性能考虑
- 异步模式减少了CPU阻塞时间，提高了系统响应性
- 但在高频率读取场景下，同步模式可能更简单高效

## 7. 兼容性说明

- 当 `SC7A20_ASYNC_SUPPORT=0` 时，所有异步代码都不会编译，不影响现有同步代码
- 同步接口 (`sc7a20_read_acceleration`, `sc7a20_write_register` 等) 始终可用
- 可以在同一项目中混合使用同步和异步接口（但不推荐对同一设备实例混用）

## 8. 调试技巧

### 8.1 启用调试输出
```c
#define DEBUG_ASYNC 1

#ifdef DEBUG_ASYNC
#define ASYNC_DEBUG(fmt, ...) printf("[ASYNC] " fmt, ##__VA_ARGS__)
#else
#define ASYNC_DEBUG(fmt, ...)
#endif
```

### 8.2 监控异步状态
```c
void print_async_status(sc7a20_handle_t handle)
{
    const char* states[] = {"IDLE", "WRITE_REG_ADDR", "WRITE_DATA", "READ_DATA", "COMPLETE"};
    printf("Async state: %s\n", states[handle->async_ctx.state]);
}
```

