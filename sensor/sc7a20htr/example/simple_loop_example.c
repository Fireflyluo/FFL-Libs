/**
 ******************************************************************************
 * @file    simple_loop_example.c
 * @brief   SC7A20HTR 异步操作简单主循环示例
 ******************************************************************************
 * @note    本文件演示如何在普通主循环中使用SC7A20HTR的异步接口，
 *          不依赖TMOS或其他RTOS。
 *
 ******************************************************************************
 */

#include "platform.h"
#include "debug.h"  // 用于printf等调试输出

#if SC7A20_ASYNC_SUPPORT

// 全局变量
static sc7a20_accel_data_t g_accel_data;
static volatile bool g_accel_data_ready = false;
static volatile sc7a20_status_t g_accel_operation_status = SC7A20_OK;

// 延时函数（需要根据实际硬件平台实现）
void delay_ms(uint32_t ms)
{
    // 这里假设有一个延时函数，实际实现可能不同
    // 例如：for (volatile uint32_t i = 0; i < ms * 1000; i++);
    Delay_Ms(ms);
}

// 简单的毫秒级时间获取函数（用于超时检测）
uint32_t get_millis(void)
{
    // 这里假设有一个获取系统时间的函数
    // 在实际应用中，可以使用SysTick或其他定时器
    static uint32_t counter = 0;
    return counter++;
}

// 异步读取完成回调
void accel_read_callback(sc7a20_handle_t handle, sc7a20_status_t status)
{
    if (status == SC7A20_OK) {
        g_accel_data_ready = true;
        printf("加速度数据读取完成!\n");
    } else {
        printf("加速度数据读取失败: %d\n", status);
    }
    g_accel_operation_status = status;
}

// 主初始化函数
int main_init(void)
{
    printf("初始化SC7A20HTR加速度计...\n");
    
    // 初始化加速度计（异步模式）
    int ret = accel_init_async();
    if (ret != 0) {
        printf("加速度计初始化失败!\n");
        return -1;
    }
    
    printf("加速度计初始化成功!\n");
    return 0;
}

// 主循环函数
void main_loop(void)
{
    static uint32_t last_read_time = 0;
    const uint32_t read_interval = 100; // 每100ms读取一次
    
    while (1) {
        uint32_t current_time = get_millis();
        
        // 定期触发异步读取
        if (current_time - last_read_time >= read_interval) {
            // 检查是否有正在进行的操作
            sc7a20_handle_t handle = get_accel_handle();
            if (handle->async_ctx.state == SC7A20_ASYNC_IDLE) {
                // 启动异步读取
                sc7a20_status_t status = sc7a20_read_acceleration_async(handle, &g_accel_data, accel_read_callback);
                if (status == SC7A20_OK) {
                    printf("启动异步读取...\n");
                } else {
                    printf("启动异步读取失败: %d\n", status);
                }
                last_read_time = current_time;
            }
        }
        
        // 检查异步操作是否完成
        if (g_accel_data_ready) {
            // 处理加速度数据
            printf("加速度数据: X=%fg, Y=%fg, Z=%fg\n", 
                   g_accel_data.x_g, g_accel_data.y_g, g_accel_data.z_g);
            printf("原始数据: X=%d, Y=%d, Z=%d\n",
                   g_accel_data.x, g_accel_data.y, g_accel_data.z);
            
            g_accel_data_ready = false;
        }
        
        // 简单的延时，避免CPU占用过高
        delay_ms(10);
    }
}

// I2C回调函数实现（需要在应用层定义）
// 这些函数会被I2C中断调用

void i2c_master_tx_cplt_callback(i2c_num_t i2c_num)
{
    // 调用平台层的写完成回调
    extern void i2c_write_complete_callback(i2c_num_t i2c_num);
    i2c_write_complete_callback(i2c_num);
}

void i2c_master_rx_cplt_callback(i2c_num_t i2c_num)
{
    // 调用平台层的读完成回调
    extern void i2c_read_complete_callback(i2c_num_t i2c_num);
    i2c_read_complete_callback(i2c_num);
}

void i2c_error_callback(i2c_num_t i2c_num, i2c_ErrCode_t errcode)
{
    // 调用平台层的错误回调
    extern void i2c_error_callback_impl(i2c_num_t i2c_num, i2c_ErrCode_t errcode);
    i2c_error_callback_impl(i2c_num, errcode);
}

// 简单的main函数示例
int main(void)
{
    // 系统初始化（根据实际平台调整）
    // SystemInit();
    // UART_Init(); // 用于调试输出
    
    // 初始化
    if (main_init() != 0) {
        printf("初始化失败，程序退出!\n");
        return -1;
    }
    
    // 进入主循环
    main_loop();
    
    return 0;
}

#endif // SC7A20_ASYNC_SUPPORT