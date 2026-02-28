/**
 ******************************************************************************
 * @file    async_example.c
 * @brief   SC7A20HTR 异步操作使用示例
 ******************************************************************************
 * @note    本文件演示如何在TMOS任务中使用SC7A20HTR的异步接口
 *
 ******************************************************************************
 */

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
        // 发送事件给TMOS任务
        tmos_set_event(accel_task_id, ACCEL_DATA_READY_EVENT);
    } else {
        // 处理错误
        printf("加速度计读取失败: %d\n", status);
    }
}

// TMOS任务函数
void accel_task_process(uint8_t task_id, uint16_t events)
{
    accel_task_id = task_id;
    
    if (events & ACCEL_READ_TRIGGER_EVENT) {
        // 触发异步读取
        sc7a20_handle_t handle = get_accel_handle();
        sc7a20_status_t status = sc7a20_read_acceleration_async(handle, &g_accel_data, accel_read_callback);
        
        if (status != SC7A20_OK) {
            printf("启动异步读取失败: %d\n", status);
        }
        // 不会阻塞，立即返回
    }
    
    if (events & ACCEL_DATA_READY_EVENT) {
        if (g_accel_data_ready) {
            // 处理加速度数据
            printf("X=%fg, Y=%fg, Z=%fg\n", 
                   g_accel_data.x_g, g_accel_data.y_g, g_accel_data.z_g);
            g_accel_data_ready = false;
            
            // 可以再次触发下一次读取
            tmos_set_event(accel_task_id, ACCEL_READ_TRIGGER_EVENT);
        }
    }
}

// 应用初始化函数
int app_init(void)
{
    // 初始化加速度计（异步模式）
    int ret = accel_init_async();
    if (ret != 0) {
        return -1;
    }
    
    // 注册I2C回调函数（需要在应用层实现）
    // 这些是弱函数，需要在main.c或其他地方实现
    // i2c_master_tx_cplt_callback = i2c_write_complete_callback;
    // i2c_master_rx_cplt_callback = i2c_read_complete_callback;
    // i2c_error_callback = i2c_error_callback;
    
    return 0;
}

#endif // SC7A20_ASYNC_SUPPORT