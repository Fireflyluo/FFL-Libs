/**
 ******************************************************************************
 * @file    sc7a20_platform.c
 * @brief   platform 平台相关代码
 ******************************************************************************
 * @note    本文件实现平台抽象层。根据宏定义切换不同平台实现。
 *
 ******************************************************************************
 */

#include "platform.h"

#ifdef CH32v208

#include "drv_i2c.h"
#include "debug.h"

// 静态分配设备结构体
static sc7a20_dev_t g_accel_dev;

// 全局设备句柄引用（用于回调）
static sc7a20_handle_t g_current_handle = NULL;

// 硬件操作函数实现（同步模式）
static sc7a20_status_t i2c_write_reg(uint8_t reg, const uint8_t *data, uint16_t len)
{
    // 实际实现I2C写操作
    int ret = i2c_write_register(I2C_NUM_1, SC7A20_I2C_ADDR_H, reg, (uint8_t *)data, len);
    return (ret == 0) ? SC7A20_OK : SC7A20_COMM_ERROR;
}

static sc7a20_status_t i2c_read_reg(uint8_t reg, uint8_t *data, uint16_t len)
{
    // 实际实现I2C读操作
    int ret = i2c_read_register(I2C_NUM_1, SC7A20_I2C_ADDR_H, reg, (uint8_t *)data, len);
    return (ret == 0) ? SC7A20_OK : SC7A20_COMM_ERROR;
}

#if SC7A20_ASYNC_SUPPORT
// 获取全局设备句柄（用于回调）
sc7a20_handle_t get_accel_handle(void)
{
    return &g_accel_dev;
}

// 设置当前操作句柄（用于回调）
void set_current_handle(sc7a20_handle_t handle)
{
    g_current_handle = handle;
}

// I2C写完成回调实现
void i2c_write_complete_callback(i2c_num_t i2c_num)
{
    if (g_current_handle && g_current_handle->async_ctx.callback) {
        g_current_handle->async_ctx.callback(g_current_handle, SC7A20_OK);
    }
    g_current_handle = NULL;
}

// I2C读完成回调实现
void i2c_read_complete_callback(i2c_num_t i2c_num)
{
    if (g_current_handle && g_current_handle->async_ctx.callback) {
        // 数据已在I2C驱动中填充到缓冲区
        g_current_handle->async_ctx.callback(g_current_handle, SC7A20_OK);
    }
    g_current_handle = NULL;
}

// I2C错误回调实现
void i2c_error_callback_impl(i2c_num_t i2c_num, i2c_ErrCode_t errcode)
{
    if (g_current_handle && g_current_handle->async_ctx.callback) {
        g_current_handle->async_ctx.callback(g_current_handle, SC7A20_COMM_ERROR);
    }
    g_current_handle = NULL;
}
#endif // SC7A20_ASYNC_SUPPORT

// 硬件操作接口（同步模式）
static sc7a20_ops_t g_accel_ops_sync = {
    .write = i2c_write_reg,
    .read = i2c_read_reg,
    .delay_ms = Delay_Ms,
#if SC7A20_ASYNC_SUPPORT
    .write_async = NULL,
    .read_async = NULL,
    .is_busy = NULL,
#endif
    .user_data = NULL
};

#if SC7A20_ASYNC_SUPPORT
// 硬件操作接口（异步模式）
static sc7a20_ops_t g_accel_ops_async = {
    .write = i2c_write_reg,
    .read = i2c_read_reg,
    .delay_ms = Delay_Ms,
    .write_async = i2c_write_reg_async,
    .read_async = i2c_read_reg_async,
    .is_busy = i2c_is_busy,
    .user_data = NULL
};
#endif // SC7A20_ASYNC_SUPPORT

// 初始化函数（同步模式）
int accel_init(void)
{
    sc7a20_config_t config = {
        .i2c_addr = SC7A20_I2C_ADDR_H, // 根据硬件连接选择地址
        .range = SC7A20_ACCEL_FS_2G,   // ±2g量程
        .odr = SC7A20_ACCEL_ODR_50HZ,  // 50Hz输出率
        .enable_axis = {1, 1, 1},      // 三轴都使能
        .block_data_update = true,     // 使能块数据更新
        .high_resolution_mode = true,  // 高分辨率模式
        .low_power_mode = false        // 正常功耗模式
    };

    sc7a20_status_t status = sc7a20_init(&g_accel_dev, &g_accel_ops_sync, &config);

    if (status != SC7A20_OK)
    {
        printf("加速度计初始化失败: %d\n", status);
        return -1;
    }

    printf("加速度计初始化成功\n");
    return 0;
}

#if SC7A20_ASYNC_SUPPORT
// 初始化函数（异步模式）
int accel_init_async(void)
{
    // 配置I2C为中断模式
    i2c_config_t i2c_config = {
        .clock_speed = 400000,
        .duty_cycle = I2C_DutyCycle_2,
        .own_address = 0,
        .enable_ack = true,
        .is_7_bit_address = true
    };
    
    // 初始化I2C为中断模式
    int ret = i2c_init(I2C_NUM_1, &i2c_config);
    if (ret != 0) {
        printf("I2C初始化失败\n");
        return -1;
    }
    
    sc7a20_config_t config = {
        .i2c_addr = SC7A20_I2C_ADDR_H,
        .range = SC7A20_ACCEL_FS_2G,
        .odr = SC7A20_ACCEL_ODR_50HZ,
        .enable_axis = {1, 1, 1},
        .block_data_update = true,
        .high_resolution_mode = true,
        .low_power_mode = false
    };
    
    sc7a20_status_t status = sc7a20_init(&g_accel_dev, &g_accel_ops_async, &config);
    
    if (status != SC7A20_OK) {
        printf("加速度计异步初始化失败: %d\n", status);
        return -1;
    }
    
    printf("加速度计异步初始化成功\n");
    return 0;
}
#endif // SC7A20_ASYNC_SUPPORT

// 读取加速度数据示例
void read_acceleration_data(void)
{
    sc7a20_accel_data_t accel_data;
    sc7a20_status_t status;

    // 检查数据是否就绪
    bool data_ready = true;
    status = sc7a20_is_data_ready(&g_accel_dev, &data_ready);
    if (status == SC7A20_OK && data_ready)
    {
        // 读取加速度数据
        status = sc7a20_read_acceleration(&g_accel_dev, &accel_data);

        if (status == SC7A20_OK)
        {
            printf("加速度数据: X=%fg, Y=%fg, Z=%fg\n",
                   accel_data.x_g, accel_data.y_g, accel_data.z_g);
            printf("原始数据: X=%d, Y=%d, Z=%d\n",
                   accel_data.x, accel_data.y, accel_data.z);
        }
    }
}

#if SC7A20_ASYNC_SUPPORT
// 检查I2C是否忙
static bool i2c_is_busy(void)
{
    return i2c_is_busy(I2C_NUM_1);
}

// 异步写寄存器实现
static sc7a20_status_t i2c_write_reg_async(uint8_t reg, 
                                          const uint8_t *data, 
                                          uint16_t len,
                                          void (*callback)(void *user_data, sc7a20_status_t status))
{
    // 构建包含寄存器地址的数据缓冲区
    static uint8_t write_buffer[I2C_MAX_WRITE_LEN];
    if (len + 1 > I2C_MAX_WRITE_LEN) {
        return SC7A20_INVALID_PARAM;
    }
    
    write_buffer[0] = reg;
    memcpy(&write_buffer[1], data, len);
    
    // 调用I2C异步写函数
    int ret = i2c_write_async(I2C_NUM_1, SC7A20_I2C_ADDR_H, write_buffer, len + 1);
    if (ret != 0) {
        return SC7A20_COMM_ERROR;
    }
    
    return SC7A20_OK;
}

// 异步读寄存器实现
static sc7a20_status_t i2c_read_reg_async(uint8_t reg, 
                                         uint8_t *data, 
                                         uint16_t len,
                                         void (*callback)(void *user_data, sc7a20_status_t status))
{
    // 先写寄存器地址，再读数据
    int ret = i2c_read_register_async(I2C_NUM_1, SC7A20_I2C_ADDR_H, reg, data, len);
    if (ret != 0) {
        return SC7A20_COMM_ERROR;
    }
    
    return SC7A20_OK;
}

#endif // SC7A20_ASYNC_SUPPORT

// I2C回调函数的外部声明（需要在应用层实现）
extern void i2c_master_tx_cplt_callback(i2c_num_t i2c_num);
extern void i2c_master_rx_cplt_callback(i2c_num_t i2c_num);
extern void i2c_error_callback(i2c_num_t i2c_num, i2c_ErrCode_t errcode);

#endif // CH32v208