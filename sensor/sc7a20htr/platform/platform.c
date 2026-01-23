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

#include "i2c.h"


// 硬件操作函数实现
static sc7a20_status_t i2c_write_reg(uint8_t reg, const uint8_t *data, uint16_t len)
{
    // 实际实现I2C写操作
    // I2C_Master_Transmit_Polling(0x19 << 1, reg, (uint8_t *)data, len);
    i2c_write_register(I2C_NUM_1, 0x19, reg, (uint8_t *)data, len);

    return SC7A20_OK;
}

static sc7a20_status_t i2c_read_reg(uint8_t reg, uint8_t *data, uint16_t len)
{
    // 实际实现I2C读操作
    // I2C_Master_Receive_Polling(0x19 << 1, reg, (uint8_t *)data, len);
    i2c_read_register(I2C_NUM_1, 0x19, reg, (uint8_t *)data, len);
    return SC7A20_OK;
}
// 硬件操作接口
static sc7a20_ops_t g_accel_ops = {
    .write = i2c_write_reg,
    .read = i2c_read_reg,
    .delay_ms = Delay_Ms,
    .user_data = NULL};

// 静态分配设备结构体
static sc7a20_dev_t g_accel_dev;
// 初始化函数
int accel_init(void)
{
    sc7a20_config_t config = {
        .i2c_addr = SC7A20_I2C_ADDR_H, // 根据硬件连接选择地址
        .range = SC7A20_ACCEL_FS_2G,   // ±4g量程
        .odr = SC7A20_ACCEL_ODR_50HZ,  // 100Hz输出率
        .enable_axis = {1, 1, 1},      // 三轴都使能
        .block_data_update = true,     // 使能块数据更新
        .high_resolution_mode = true,  // 正常分辨率
        .low_power_mode = false        // 正常功耗模式
    };

    sc7a20_status_t status = sc7a20_init(&g_accel_dev, &g_accel_ops, &config);

    if (status != SC7A20_OK)
    {
        printf("加速度计初始化失败: %d\n", status);
        return -1;
    }

    printf("加速度计初始化成功\n");
    return 0;
}
// 读取加速度数据示例
void read_acceleration_data(void)
{
    sc7a20_accel_data_t accel_data;
    sc7a20_status_t status;

    // 检查数据是否就绪
    bool data_ready = 1;
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
#endif // CH32v208
