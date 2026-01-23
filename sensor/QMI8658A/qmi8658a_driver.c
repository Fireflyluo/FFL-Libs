/*-----------------------------------------------File Info------------------------------------------------
** File Name:               qmi8658a_driver.c  
** Last modified date:      2025.8.18
** Last version:            V0.1
** Description:             QMI8658A传感器驱动实现
**
**--------------------------------------------------------------------------------------------------------            
** Created date:            2025.7.1
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            统一传感器接口定义的各个函数（init, read_data, set_range, set_odr, sleep, wakeup）
**                          QMI8658A 传感器驱动的核心实现，提供初始化、数据读取、量程/ODR配置等具体功能。                        
**--------------------------------------------------------------------------------------------------------*/

#include "qmi8658a_driver.h"
#include "qmi8658a_reg.h"
#include <math.h>

// 定义圆周率（兼容性方案）
#ifndef M_PI
#define M_PI 3.1415926535f
#endif

/* 设备上下文 */
static qmi8658a_driver_ctx_t ctx;

/* 私有函数声明 */
static int32_t qmi8658a_read_data(sensor_data_t *data);
static int32_t qmi8658a_set_range(gyro_range_t range);
static int32_t qmi8658a_set_odr(gyro_odr_t odr);
static int32_t qmi8658a_sleep(void);
static int32_t qmi8658a_wakeup(void);
static float convert_accel(int16_t raw);
static float convert_gyro(int16_t raw);

/**
  * @brief  初始化设备
  * @param  hardware_handle: 硬件抽象层句柄(I2C/SPI)
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_init(void *hardware_handle)
{
    // 初始化寄存器操作上下文
    ctx.reg_ctx.read_reg = qmi8658a_i2c_read;
    ctx.reg_ctx.write_reg = qmi8658a_i2c_write;
    ctx.reg_ctx.handle = hardware_handle;
    
    // 验证设备ID
    uint8_t who_am_i;
    if (qmi8658a_device_id_get(&ctx.reg_ctx, &who_am_i) != 0 || 
        who_am_i != QMI8658A_ID) {
          
        return -1; // 设备未找到
    }
    
    // 软复位
    uint8_t reset_cmd = 0xB0;
    qmi8658a_write_reg(&ctx.reg_ctx, QMI8658A_RESET, &reset_cmd, 1);
    HAL_Delay(50);
    
    // 添加CTRL1配置：启用地址自动递增
    uint8_t ctrl1_val = 0x40; // 0x40 = 0b01000000，设置Bit 6 (ADDR_AI=1)，其他位默认
    qmi8658a_write_reg(&ctx.reg_ctx, QMI8658A_CTRL1, &ctrl1_val, 1);
    
//    qmi8658a_accel_enable(&ctx.reg_ctx, 1);
//    qmi8658a_gyro_enable(&ctx.reg_ctx, 1);
    uint8_t ctrl7_val = 0x03;
    qmi8658a_write_reg(&ctx.reg_ctx, QMI8658A_CTRL7, &ctrl7_val, 1);


    // 加速度计配置
//    qmi8658a_accel_data_rate_set(&ctx.reg_ctx, QMI8658A_ACCEL_ODR_104HZ);
//    qmi8658a_accel_full_scale_set(&ctx.reg_ctx, QMI8658A_ACCEL_FS_4G);
    ctx.current_accel_fs = QMI8658A_ACCEL_FS_4G;
    uint8_t ctrl2_val = 0x94;//启用自检,4G,ODR 500Hz
    qmi8658a_write_reg(&ctx.reg_ctx, QMI8658A_CTRL2, &ctrl2_val, 1);

    // 陀螺仪配置   
//    qmi8658a_gyro_data_rate_set(&ctx.reg_ctx, QMI8658A_GYRO_ODR_104HZ);
//    qmi8658a_gyro_full_scale_set(&ctx.reg_ctx, QMI8658A_GYRO_FS_512DPS);
    ctx.current_gyro_range = GYRO_RANGE_500DPS;
    uint8_t ctrl3_val = 0xD4;
    qmi8658a_write_reg(&ctx.reg_ctx, QMI8658A_CTRL3, &ctrl3_val, 1);
    
    // 滤波器配置
     uint8_t ctrl5_val = 0x55;
    qmi8658a_write_reg(&ctx.reg_ctx, QMI8658A_CTRL5, &ctrl5_val, 1);

    return 0;
}

/**
  * @brief  读取传感器数据
  * @param  data: 存储传感器数据的结构体指针
  * @retval 0: 成功；非0: 失败
  */
static int32_t qmi8658a_read_data(sensor_data_t *data)
{
    uint8_t raw_data[14]; // 6字节加速度 + 6字节陀螺仪 
    // 检查数据可用状态
    uint8_t status;
    qmi8658a_read_reg(&ctx.reg_ctx, QMI8658A_STATUS0, &status, 1);
    if (!(status & 0x03)) return -1; // 数据未就绪
    
    // 一次性读取所有数据
    if (qmi8658a_read_reg(&ctx.reg_ctx, QMI8658A_OUT_TEMP_L, raw_data, 14) != 0) {
        return -1;
    }
        // 解析温度数据
    int16_t temp = (int16_t)(raw_data[1] << 8 | raw_data[0]);
    
    // 解析加速度数据 (小端格式)
    int16_t accel_x = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    int16_t accel_y = (int16_t)((raw_data[5] << 8) | raw_data[4]);
    int16_t accel_z = (int16_t)((raw_data[7] << 8) | raw_data[6]);
    
    // 解析陀螺仪数据
    int16_t gyro_x = (int16_t)((raw_data[9] << 8) | raw_data[8]);
    int16_t gyro_y = (int16_t)((raw_data[11] << 8) | raw_data[10]);
    int16_t gyro_z = (int16_t)((raw_data[13] << 8) | raw_data[12]);
    
//    printf("%d %d %d %d %d %d\n",accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z);


    // 转换为物理量
    data->accel[0] = convert_accel(accel_x);
    data->accel[1] = convert_accel(accel_y);
    data->accel[2] = convert_accel(accel_z);
    
    data->gyro[0] = convert_gyro(gyro_x);
    data->gyro[1] = convert_gyro(gyro_y);
    data->gyro[2] = convert_gyro(gyro_z);
     
    // 温度转换 (数据手册公式)
    data->temp = temp / 256.0f;
    
    return 0;
}

/**
  * @brief  设置陀螺仪量程
  * @param  range: 目标量程枚举值
  * @retval 0: 成功；非0: 失败
  */
static int32_t qmi8658a_set_range(gyro_range_t range)
{
    qmi8658a_gyro_fs_t fs;
    switch(range) {
        case GYRO_RANGE_250DPS: fs = QMI8658A_GYRO_FS_256DPS; break;
        case GYRO_RANGE_500DPS: fs = QMI8658A_GYRO_FS_512DPS; break;
        case GYRO_RANGE_1000DPS: fs = QMI8658A_GYRO_FS_1024DPS; break;
        case GYRO_RANGE_2000DPS: fs = QMI8658A_GYRO_FS_2048DPS; break;
        default: return -1;
    }
    
    if (qmi8658a_gyro_full_scale_set(&ctx.reg_ctx, fs) != 0) {
        return -1;
    }
    
    ctx.current_gyro_range = range;
    return 0;
}

/**
  * @brief  设置输出数据率
  * @param  odr: 目标ODR枚举值
  * @retval 0: 成功；非0: 失败
  */
static int32_t qmi8658a_set_odr(gyro_odr_t odr)
{
    qmi8658a_gyro_odr_t g_odr;
    switch(odr) {
        case GYRO_ODR_100HZ: g_odr = QMI8658A_GYRO_ODR_112_1HZ; break;
        case GYRO_ODR_200HZ: g_odr = QMI8658A_GYRO_ODR_224_2HZ; break;
        case GYRO_ODR_400HZ: g_odr = QMI8658A_GYRO_ODR_448_4HZ; break;
        case GYRO_ODR_800HZ: g_odr = QMI8658A_GYRO_ODR_896_8HZ; break;
        default: return -1;
    }
    
    if (qmi8658a_gyro_data_rate_set(&ctx.reg_ctx, g_odr) != 0) {
        return -1;
    }
    
    return 0;
}

/**
  * @brief  进入低功耗模式
  * @retval 0: 成功；非0: 失败
  */
static int32_t qmi8658a_sleep(void)
{
    qmi8658a_ctrl7_t ctrl7 = {0};
    return qmi8658a_write_reg(&ctx.reg_ctx, QMI8658A_CTRL7, (uint8_t*)&ctrl7, 1);
}

/**
  * @brief  唤醒设备
  * @retval 0: 成功；非0: 失败
  */
static int32_t qmi8658a_wakeup(void)
{
    
    qmi8658a_ctrl7_t ctrl7 = {
        .bit = {
            .aEN = 1,      // 启用加速度计
            .gEN = 1,      // 启用陀螺仪
            .syncSmpl = 1  // 启用同步采样模式
        }
    };
    
    return qmi8658a_write_reg(&ctx.reg_ctx, QMI8658A_CTRL7, &ctrl7.reg, 1);
}

/**
  * @brief  加速度原始数据转换
  * @param  raw: 原始加速度值
  * @retval 转换后的加速度值(m/s2)
  */
static float convert_accel(int16_t raw)
{
    // 根据当前量程动态计算灵敏度
    float full_scale_g;
    switch(ctx.current_accel_fs) {
        case QMI8658A_ACCEL_FS_2G:  full_scale_g = 2.0f; break;
        case QMI8658A_ACCEL_FS_4G:  full_scale_g = 4.0f; break;
        case QMI8658A_ACCEL_FS_8G:  full_scale_g = 8.0f; break;
        case QMI8658A_ACCEL_FS_16G: full_scale_g = 16.0f; break;
        default: full_scale_g = 4.0f;
    }
    
    // 灵敏度 = (量程范围 / 32768) * 重力加速度
    const float scale = (full_scale_g / 32768.0f) * 9.80665f;
    return raw * scale;
}

/**
  * @brief  陀螺仪原始数据转换
  * @param  raw: 原始陀螺仪值
  * @retval 转换后的角速度值(rad/s)
  */
static float convert_gyro(int16_t raw)
{
    // 根据当前量程动态计算灵敏度
    float full_scale_dps;
    switch(ctx.current_gyro_range) {
        case GYRO_RANGE_250DPS: full_scale_dps = 256.0f; break;
        case GYRO_RANGE_500DPS: full_scale_dps = 512.0f; break;
        case GYRO_RANGE_1000DPS: full_scale_dps = 1024.0f; break;
        case GYRO_RANGE_2000DPS: full_scale_dps = 2048.0f; break;
        default: full_scale_dps = 256.0f;
    }
    
    // 灵敏度 = (量程范围 / 32768) * (π/180)
    const float scale = (full_scale_dps / 32768.0f) * (M_PI / 180.0f);
    return raw * scale;
}

/* 导出统一接口 */
const gyro_device_t qmi8658a_device = {
    .init = qmi8658a_init,
    .read_data = qmi8658a_read_data,//读取传感器数据
    .set_range = qmi8658a_set_range,//设置陀螺仪量程
    .set_odr = qmi8658a_set_odr,    //设置输出数据率
    .sleep = qmi8658a_sleep,
    .wakeup = qmi8658a_wakeup
};
