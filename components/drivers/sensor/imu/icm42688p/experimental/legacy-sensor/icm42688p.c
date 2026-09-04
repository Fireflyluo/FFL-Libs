#include "icm42688p.h"
#include "icm42688_reg.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
/* ========================================================================== */
/*                           内部使用的宏定义                                */
/* ========================================================================== */

/* 内部寄存器操作宏 */
#define ICM42688_BANK_SELECT(dev, bank)                                                                 \
    do                                                                                                  \
    {                                                                                                   \
        if ((bank) > ICM42688_BANK_MAX)                                                                 \
        {                                                                                               \
            return ICM42688_ERR_INVALID_PARAM;                                                          \
        }                                                                                               \
        icm42688_reg_bank_sel_t bank_sel = {.bits.USER_BANK = (uint8_t)(bank)};                         \
        icm42688_err_t ret = icm42688_write_reg_internal(dev, ICM42688_REG_BANK_SEL, &bank_sel.reg, 1); \
        if (ret == ICM42688_OK)                                                                         \
        {                                                                                               \
            dev->current_bank = (bank);                                                                 \
        }                                                                                               \
    } while (0)

#define ICM42688_CHECK_INIT(dev)             \
    if (!dev || !dev->initialized)           \
    {                                        \
        return ICM42688_ERR_NOT_INITIALIZED; \
    }

#define ICM42688_CHECK_PARAM(dev)          \
    if (!dev)                              \
    {                                      \
        return ICM42688_ERR_INVALID_PARAM; \
    }

/* 超时保护宏 */
#define ICM42688_TIMEOUT_CHECK(start, timeout) \
    (dev->comm.get_tick ? (dev->comm.get_tick() - start > timeout) : false)
/* ========================================================================== */
/*                           内部静态函数声明                                */
/* ========================================================================== */

/* 寄存器操作内部函数 */
static icm42688_err_t icm42688_write_reg_internal(icm42688_device_t *dev,
                                                  uint8_t reg_addr,
                                                  const uint8_t *data,
                                                  uint16_t len);
static icm42688_err_t icm42688_read_reg_internal(icm42688_device_t *dev, uint8_t reg_addr, uint8_t *data, uint16_t len);

/* 传感器配置内部函数 */
static icm42688_err_t icm42688_config_gyro_internal(icm42688_device_t *dev);
static icm42688_err_t icm42688_config_accel_internal(icm42688_device_t *dev);
static icm42688_err_t icm42688_config_fifo_internal(icm42688_device_t *dev);
static icm42688_err_t icm42688_config_interrupt_internal(icm42688_device_t *dev);

/* 数据转换内部函数 */
static float icm42688_convert_accel_raw_to_ms2(int16_t raw, icm42688_accel_fs_t fs);
static float icm42688_convert_gyro_raw_to_rads(int16_t raw, icm42688_gyro_fs_t fs);
static float icm42688_convert_temp_raw_to_celsius(int16_t raw);

/* 校验和工具函数 */
static uint8_t icm42688_calculate_checksum(const uint8_t *data, uint16_t len);

/* ========================================================================== */
/*                           内部使用的常量数据                              */
/* ========================================================================== */
/**
 * @brief 陀螺仪灵敏度查找表 (LSB/dps)
 * @note 根据满量程范围对应的灵敏度值
 */
static const float gyro_sensitivity_lut[] = {
    [ICM42688_GYRO_FS_2000DPS] = 16.384f,     // ±2000 dps
    [ICM42688_GYRO_FS_1000DPS] = 32.768f,     // ±1000 dps
    [ICM42688_GYRO_FS_500DPS] = 65.536f,      // ±500 dps
    [ICM42688_GYRO_FS_250DPS] = 131.072f,     // ±250 dps
    [ICM42688_GYRO_FS_125DPS] = 262.144f,     // ±125 dps
    [ICM42688_GYRO_FS_62_5DPS] = 524.288f,    // ±62.5 dps
    [ICM42688_GYRO_FS_31_25DPS] = 1048.576f,  // ±31.25 dps
    [ICM42688_GYRO_FS_15_625DPS] = 2097.152f, // ±15.625 dps
};

/**
 * @brief 加速度计灵敏度查找表 (LSB/g)
 * @note 根据满量程范围对应的灵敏度值
 */
static const float accel_sensitivity_lut[] = {
    [ICM42688_ACCEL_FS_16G] = 2048.0f, // ±16g
    [ICM42688_ACCEL_FS_8G] = 4096.0f,  // ±8g
    [ICM42688_ACCEL_FS_4G] = 8192.0f,  // ±4g
    [ICM42688_ACCEL_FS_2G] = 16384.0f, // ±2g
};

/**
 * @brief ODR值对应的实际频率表 (Hz)
 * @note 根据寄存器值映射到实际输出频率
 */
static const uint32_t odr_frequency_lut[] = {
    [ICM42688_ODR_32000HZ] = 32000,
    [ICM42688_ODR_16000HZ] = 16000,
    [ICM42688_ODR_8000HZ] = 8000,
    [ICM42688_ODR_4000HZ] = 4000,
    [ICM42688_ODR_2000HZ] = 2000,
    [ICM42688_ODR_1000HZ] = 1000,
    [ICM42688_ODR_500HZ] = 500,
    [ICM42688_ODR_200HZ] = 200,
    [ICM42688_ODR_100HZ] = 100,
    [ICM42688_ODR_50HZ] = 50,
    [ICM42688_ODR_25HZ] = 25,
    [ICM42688_ODR_12_5HZ] = 12,
    [ICM42688_ODR_6_25HZ] = 6,
    [ICM42688_ODR_3_125HZ] = 3,
    [ICM42688_ODR_1_5625HZ] = 1,
};

/* ========================================================================== */
/*                           内部寄存器操作函数                              */
/* ========================================================================== */

/**
 * @brief 内部寄存器写函数
 * @param dev 设备句柄
 * @param reg_addr 寄存器地址
 * @param data 要写入的数据
 * @param len 数据长度
 * @return 错误码
 * @note 此函数处理Bank切换和通信错误重试
 */
static icm42688_err_t icm42688_write_reg_internal(icm42688_device_t *dev,
                                                  uint8_t reg_addr,
                                                  const uint8_t *data,
                                                  uint16_t len)
{
    icm42688_err_t ret;
    uint8_t retry_count = 3;

    if (!dev || !dev->comm.write_reg)
    {
        return ICM42688_ERR_INVALID_PARAM;
    }

    /* 重试机制，提高通信可靠性 */
    while (retry_count--)
    {
        ret = dev->comm.write_reg(dev->comm.dev_addr, reg_addr, data, len);
        if (ret == ICM42688_OK)
        {
            break;
        }

        /* 通信失败时延时后重试 */
        if (dev->comm.delay_ms)
        {
            dev->comm.delay_ms(1);
        }
    }

    return ret;
}
/**
 * @brief 内部寄存器读函数
 * @param dev 设备句柄
 * @param reg_addr 寄存器地址
 * @param data 读取数据缓冲区
 * @param len 数据长度
 * @return 错误码
 * @note 此函数处理Bank切换和通信错误重试
 */
static icm42688_err_t icm42688_read_reg_internal(icm42688_device_t *dev,
                                                 uint8_t reg_addr,
                                                 uint8_t *data,
                                                 uint16_t len)
{
    icm42688_err_t ret;
    uint8_t retry_count = 3;

    if (!dev || !dev->comm.read_reg)
    {
        return ICM42688_ERR_INVALID_PARAM;
    }

    /* 重试机制，提高通信可靠性 */
    while (retry_count--)
    {
        ret = dev->comm.read_reg(dev->comm.dev_addr, reg_addr, data, len);
        if (ret == ICM42688_OK)
        {
            break;
        }

        /* 通信失败时延时后重试 */
        if (dev->comm.delay_ms)
        {
            dev->comm.delay_ms(1);
        }
    }

    return ret;
}
/**
 * @brief 切换到指定Bank
 * @param dev 设备句柄
 * @param bank 目标Bank
 * @return 错误码
 * @note 如果已经在目标Bank，则不会重复切换
 */
static icm42688_err_t icm42688_switch_bank(icm42688_device_t *dev, icm42688_bank_t bank)
{
    if (!dev)
    {
        return ICM42688_ERR_INVALID_PARAM;
    }

    /* 如果已经在目标Bank，直接返回 */
    if (dev->current_bank == bank)
    {
        return ICM42688_OK;
    }

    /* 切换Bank */
    icm42688_reg_bank_sel_t bank_sel = {.bits.USER_BANK = bank};
    icm42688_err_t ret = icm42688_write_reg_internal(dev, ICM42688_REG_BANK_SEL,
                                                     &bank_sel.reg, 1);
    if (ret == ICM42688_OK)
    {
        dev->current_bank = bank;
    }

    return ret;
}
/* ========================================================================== */
/*                           设备初始化和管理函数                           */
/* ========================================================================== */

/**
 * @brief 初始化ICM-42688设备
 * @param dev 设备句柄指针
 * @param comm_config 通信配置
 * @return 错误码
 * @note 此函数会执行设备复位、ID验证和默认配置
 */
icm42688_err_t icm42688_init(icm42688_device_t *dev,
                             const icm42688_comm_config_t *comm_config)
{
    ICM42688_CHECK_PARAM(dev);
    ICM42688_CHECK_PARAM(comm_config);

    /* 检查必要的函数指针 */
    if (!comm_config->write_reg || !comm_config->read_reg || !comm_config->delay_ms)
    {
        return ICM42688_ERR_INVALID_PARAM;
    }

    /* 初始化设备结构体 */
    memset(dev, 0, sizeof(icm42688_device_t));
    dev->comm = *comm_config;
    dev->current_bank = ICM42688_BANK0; // 默认Bank0

    /* 验证设备ID */
    uint8_t whoami;
    icm42688_err_t ret = icm42688_read_reg_internal(dev, ICM42688_REG_WHO_AM_I,
                                                    &whoami, 2);

    if (ret != ICM42688_OK)
        return ret;
    if (whoami != ICM42688_WHO_AM_I_ID)
        return ICM42688_ERR_NOT_READY;

    dev->whoami = whoami;

    /* 执行软件复位 */
    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);
    icm42688_reg_device_config_t dev_config = {
        .bits.SOFT_RESET = 1};
    // dev_config.reg = 0x01;   //等价于
    ret = icm42688_write_reg_internal(dev, ICM42688_REG_DEVICE_CONFIG,
                                      &dev_config.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    /* 等待复位完成 */
    if (dev->comm.delay_ms)
    {
        dev->comm.delay_ms(ICM42688_RESET_DELAY_MS);
    }

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);
    /* 配置默认参数 */
    dev->config.gyro_mode = ICM42688_MODE_OFF;
    dev->config.accel_mode = ICM42688_MODE_OFF;
    dev->config.gyro_fs = ICM42688_DEFAULT_GYRO_FS;
    dev->config.accel_fs = ICM42688_DEFAULT_ACCEL_FS;
    dev->config.gyro_odr = ICM42688_DEFAULT_ODR;
    dev->config.accel_odr = ICM42688_DEFAULT_ODR;

    dev->initialized = true;
    dev->status.sensor_ready = false;

    return ICM42688_OK;
}
/**
 * @brief 反初始化设备
 * @param dev 设备句柄指针
 * @return 错误码
 * @note 此函数会关闭传感器并重置设备状态
 */
icm42688_err_t icm42688_deinit(icm42688_device_t *dev)
{
    ICM42688_CHECK_INIT(dev);

    /* 关闭所有传感器 */
    icm42688_reg_pwr_mgmt0_t pwr_mgmt0 = {
        .bits.GYRO_MODE = ICM42688_MODE_OFF,
        .bits.ACCEL_MODE = ICM42688_MODE_OFF};

    icm42688_err_t ret = icm42688_write_reg_internal(dev, ICM42688_REG_PWR_MGMT0,
                                                     &pwr_mgmt0.reg, 1);

    /* 重置设备状态 */
    memset(dev, 0, sizeof(icm42688_device_t));

    return ret;
}

/**
 * @brief 复位设备
 * @param dev 设备句柄指针
 * @return 错误码
 */
icm42688_err_t icm42688_reset(icm42688_device_t *dev)
{
    ICM42688_CHECK_INIT(dev);

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    icm42688_reg_device_config_t dev_config = {
        .bits.SOFT_RESET = 1};

    return icm42688_write_reg_internal(dev, ICM42688_REG_DEVICE_CONFIG,
                                       &dev_config.reg, 1);
}

/**
 * @brief 检查设备连接状态
 * @param dev 设备句柄指针
 * @return true-设备正常，false-设备异常
 */
bool icm42688_check_connection(icm42688_device_t *dev)
{
    if (!dev || !dev->initialized)
    {
        return false;
    }

    uint8_t whoami;
    icm42688_err_t ret = icm42688_read_reg_internal(dev, ICM42688_REG_WHO_AM_I,
                                                    &whoami, 1);

    return (ret == ICM42688_OK) && (whoami == ICM42688_WHO_AM_I_ID);
}

/* ========================================================================== */
/*                           传感器配置函数                                 */
/* ========================================================================== */

/**
 * @brief 配置传感器参数
 * @param dev 设备句柄指针
 * @param config 传感器配置
 * @return 错误码
 * @note 此函数会配置陀螺仪、加速度计、FIFO和中断等所有参数
 */
icm42688_err_t icm42688_config_sensor(icm42688_device_t *dev,
                                      const icm42688_sensor_config_t *config)
{
    ICM42688_CHECK_INIT(dev);
    ICM42688_CHECK_PARAM(config);

    /* 保存配置参数 */
    dev->config = *config;

    /* 配置加速度计 */
    icm42688_err_t ret = icm42688_config_accel_internal(dev);
    if (ret != ICM42688_OK) return ret;
 
    /* 配置陀螺仪 */
    ret = icm42688_config_gyro_internal(dev);
    if (ret != ICM42688_OK) return ret;
 
    /* 配置FIFO */
    if (config->fifo_en == true)
    {
        // 需要用户根据实际需求修改中断配置
        ret = icm42688_config_fifo_internal(dev);
        if (ret != ICM42688_OK)  return ret;

    }

    /* 配置中断 */
    if (config->interrupt_en == true)
    {
        // 需要用户根据实际需求修改中断配置
        ret = icm42688_config_interrupt_internal(dev);
        if (ret != ICM42688_OK)return ret;
  
    }

    /* 配置电源 */
    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);
    icm42688_reg_pwr_mgmt0_t pwr_config = {
        .bits.TEMP_DIS = 0,
        .bits.GYRO_MODE = dev->config.gyro_mode,
        .bits.ACCEL_MODE = dev->config.accel_mode};
    icm42688_write_reg_internal(dev, ICM42688_REG_PWR_MGMT0,
                                &pwr_config.reg, 1);
    /* 等待配置生效 */
    if (dev->comm.delay_ms)
    {
        dev->comm.delay_ms(ICM42688_MODE_SWITCH_DELAY_MS);
    }

    dev->status.sensor_ready = true;

    return ICM42688_OK;
}

/**
 * @brief 内部陀螺仪配置函数
 * @param dev 设备句柄
 * @return 错误码
 */
static icm42688_err_t icm42688_config_gyro_internal(icm42688_device_t *dev)
{
    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 配置陀螺仪参数 */
    icm42688_reg_gyro_config0_t gyro_config = {
        .bits.GYRO_ODR = dev->config.gyro_odr,
        .bits.GYRO_FS_SEL = dev->config.gyro_fs};

    icm42688_err_t ret = icm42688_write_reg_internal(dev, ICM42688_REG_GYRO_CONFIG0,
                                                     &gyro_config.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    return ret;
}

/**
 * @brief 内部加速度计配置函数
 * @param dev 设备句柄
 * @return 错误码
 */
static icm42688_err_t icm42688_config_accel_internal(icm42688_device_t *dev)
{
    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 配置加速度计参数 */
    icm42688_reg_accel_config0_t accel_config = {
        .bits.ACCEL_ODR = dev->config.accel_odr,
        .bits.ACCEL_FS_SEL = dev->config.accel_fs};

    icm42688_err_t ret = icm42688_write_reg_internal(dev, ICM42688_REG_ACCEL_CONFIG0,
                                                     &accel_config.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    return ret;
}

/**
 * @brief 内部FIFO配置函数
 * @param dev 设备句柄
 * @return 错误码
 */
static icm42688_err_t icm42688_config_fifo_internal(icm42688_device_t *dev)
{
    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 配置FIFO */
    icm42688_reg_fifo_config1_t fifo_config = {
        .bits.FIFO_ACCEL_EN = dev->config.fifo_accel_en ? 1 : 0,
        .bits.FIFO_GYRO_EN = dev->config.fifo_gyro_en ? 1 : 0,
        .bits.FIFO_TEMP_EN = dev->config.fifo_temp_en ? 1 : 0};

    icm42688_err_t ret = icm42688_write_reg_internal(dev, ICM42688_REG_FIFO_CONFIG1,
                                                     &fifo_config.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    /* 配置FIFO水印 */
    if (dev->config.fifo_watermark > 0)
    {
        uint16_t watermark = dev->config.fifo_watermark;
        if (watermark > ICM42688_FIFO_SIZE)
        {
            watermark = ICM42688_FIFO_SIZE;
        }

        uint8_t watermark_regs[2] = {
            (uint8_t)((watermark >> 8) & 0x07), // FIFO_CONFIG2
            (uint8_t)(watermark & 0xFF)         // FIFO_CONFIG3
        };

        ret = icm42688_write_reg_internal(dev, ICM42688_REG_FIFO_CONFIG2,
                                          watermark_regs, 2);
    }

    return ret;
}

/**
 * @brief 内部中断配置函数
 * @param dev 设备句柄
 * @return 错误码
 * @note 此函数配置中断引脚参数、清除方式、并使能数据就绪中断到INT1引脚。
 *       默认配置：INT1高电平有效、推挽输出、锁存模式（电平触发），手动清除中断。
 */
static icm42688_err_t icm42688_config_interrupt_internal(icm42688_device_t *dev)
{
    ICM42688_CHECK_INIT(dev);                  // 检查设备初始化状态
    ICM42688_BANK_SELECT(dev, ICM42688_BANK0); // 选择Bank 0（必需步骤）

    // 1. 配置中断引脚电气特性（INT_CONFIG寄存器，Bank 0, Addr 0x14）
    icm42688_reg_int_config_t int_config = {
        .bits = {
            .INT1_POLARITY = 1,      // INT1高电平有效（0:低电平，1:高电平）
            .INT1_DRIVE_CIRCUIT = 1, // INT1推挽输出（0:开漏，1:推挽）
            .INT1_MODE = 1,          // INT1锁存模式（0:脉冲，1:锁存）- 实现电平触发
            .INT2_POLARITY = 0,      // INT2低电平有效（默认）
            .INT2_DRIVE_CIRCUIT = 0, // INT2开漏输出（默认）
            .INT2_MODE = 0           // INT2脉冲模式（默认）
        }};
    icm42688_err_t ret = icm42688_write_reg_internal(dev, ICM42688_REG_INT_CONFIG,
                                                     &int_config.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret; // 错误处理：寄存器写入失败
    }

    // 2. 配置中断清除方式（INT_CONFIG0寄存器，Bank 0, Addr 0x63）
    icm42688_reg_int_config0_t int_config0 = {
        .bits = {
            .FIFO_FULL_INT_CLEAR = 0, // FIFO满中断手动清除（0:手动，1:自动）
            .FIFO_THS_INT_CLEAR = 0,  // FIFO阈值中断手动清除
            .UI_DRDY_INT_CLEAR = 0    // 数据就绪中断手动清除
            // 保留位保持默认值0
        }};
    ret = icm42688_write_reg_internal(dev, ICM42688_REG_INT_CONFIG0,
                                      &int_config0.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    // 3. 使能中断源并映射到INT1引脚（INT_SOURCE0寄存器，Bank 0, Addr 0x65）
    icm42688_reg_int_source0_t int_source0 = {
        .bits = {
            .UI_AGC_RDY_INT1_EN = 1, // 使能数据就绪中断到INT1
            .FIFO_THS_INT1_EN = 0,   // 禁用FIFO阈值中断（根据需求调整）
            .FIFO_FULL_INT1_EN = 0,  // 禁用FIFO满中断
        }};
    ret = icm42688_write_reg_internal(dev, ICM42688_REG_INT_SOURCE0,
                                      &int_source0.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    // 4. 关键配置：确保INT_ASYNC_RESET位为0（INT_CONFIG1寄存器，Bank 0, Addr 0x64）
    icm42688_reg_int_config1_t int_config1;
    ret = icm42688_read_reg_internal(dev, ICM42688_REG_INT_CONFIG1,
                                     &int_config1.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }
    int_config1.bits.INT_ASYNC_RESET = 0; // 必须设置为0以确保中断正确操作
    ret = icm42688_write_reg_internal(dev, ICM42688_REG_INT_CONFIG1,
                                      &int_config1.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    return ICM42688_OK;
}

/* ========================================================================== */
/*                           数据读取函数                                   */
/* ========================================================================== */

/**
 * @brief 读取传感器原始数据
 * @param dev 设备句柄指针
 * @param raw_data 原始数据输出
 * @return 错误码
 */
icm42688_err_t icm42688_read_raw_data(icm42688_device_t *dev,
                                      icm42688_raw_data_t *raw_data)
{
    ICM42688_CHECK_INIT(dev);
    ICM42688_CHECK_PARAM(raw_data);

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 读取14字节传感器数据（6轴+温度） */
    uint8_t sensor_data[14];

    icm42688_err_t ret = icm42688_read_reg_internal(dev, ICM42688_REG_TEMP_DATA1,
                                                    sensor_data, 14);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    /* 解析原始数据（16位补码格式） */
    raw_data->accel_x = (int16_t)((sensor_data[2] << 8) | sensor_data[3]);
    raw_data->accel_y = (int16_t)((sensor_data[4] << 8) | sensor_data[5]);
    raw_data->accel_z = (int16_t)((sensor_data[6] << 8) | sensor_data[3]);
    raw_data->gyro_x = (int16_t)((sensor_data[8] << 8) | sensor_data[9]);
    raw_data->gyro_y = (int16_t)((sensor_data[10] << 8) | sensor_data[11]);
    raw_data->gyro_z = (int16_t)((sensor_data[12] << 8) | sensor_data[13]);
    raw_data->temperature = (int16_t)((sensor_data[0] << 8) | sensor_data[1]);

    /* 更新状态信息 */
    dev->status.data_ready = true;
    dev->sample_count++;

    return ICM42688_OK;
}

/* ========================================================================== */
/*                           数据转换函数                                   */
/* ========================================================================== */

/**
 * @brief 将原始加速度数据转换为物理量 (m/s²)
 * @param raw 原始数据
 * @param fs 满量程范围
 * @return 转换后的加速度值 (m/s²)
 */
static float icm42688_convert_accel_raw_to_ms2(int16_t raw, icm42688_accel_fs_t fs)
{
    /* 获取灵敏度值 */
    float sensitivity = accel_sensitivity_lut[fs];

    /* 转换为g值 */
    float accel_g = (float)raw / sensitivity;

    /* 转换为m/s² */
    return accel_g * ICM42688_G_TO_M_S2;
}

/**
 * @brief 将原始陀螺仪数据转换为物理量 (rad/s)
 * @param raw 原始数据
 * @param fs 满量程范围
 * @return 转换后的角速度值 (rad/s)
 */
static float icm42688_convert_gyro_raw_to_rads(int16_t raw, icm42688_gyro_fs_t fs)
{
    /* 获取灵敏度值 */
    float sensitivity = gyro_sensitivity_lut[fs];

    /* 转换为dps值 */
    float gyro_dps = (float)raw / sensitivity;

    /* 转换为rad/s */
    return gyro_dps * ICM42688_DPS_TO_RAD_S;
}

/**
 * @brief 将原始温度数据转换为摄氏度
 * @param raw 原始数据
 * @return 转换后的温度值 (°C)
 */
static float icm42688_convert_temp_raw_to_celsius(int16_t raw)
{
    /* 根据数据手册公式转换 */
    return ((float)raw / ICM42688_TEMP_SENSITIVITY) + ICM42688_TEMP_OFFSET;
}

/**
 * @brief 读取并转换传感器数据
 * @param dev 设备句柄指针
 * @param sensor_data 传感器数据输出
 * @return 错误码
 */
icm42688_err_t icm42688_read_sensor_data(icm42688_device_t *dev,
                                         icm42688_sensor_data_t *sensor_data)
{
    ICM42688_CHECK_INIT(dev);
    ICM42688_CHECK_PARAM(sensor_data);
    uint8_t int_status;
    icm42688_read_reg_internal(dev, ICM42688_REG_INT_STATUS, &int_status, 1);

    icm42688_raw_data_t raw_data;

    /* 读取原始数据 */

    icm42688_err_t ret = icm42688_read_raw_data(dev, &raw_data);

    if (ret != ICM42688_OK)
    {
        return ret;
    }

    /* 转换加速度数据 */
    sensor_data->accel_x = icm42688_convert_accel_raw_to_ms2(raw_data.accel_x,
                                                             dev->config.accel_fs);
    sensor_data->accel_y = icm42688_convert_accel_raw_to_ms2(raw_data.accel_y,
                                                             dev->config.accel_fs);
    sensor_data->accel_z = icm42688_convert_accel_raw_to_ms2(raw_data.accel_z,
                                                             dev->config.accel_fs);

    /* 转换陀螺仪数据 */
    sensor_data->gyro_x = icm42688_convert_gyro_raw_to_rads(raw_data.gyro_x,
                                                            dev->config.gyro_fs);
    sensor_data->gyro_y = icm42688_convert_gyro_raw_to_rads(raw_data.gyro_y,
                                                            dev->config.gyro_fs);
    sensor_data->gyro_z = icm42688_convert_gyro_raw_to_rads(raw_data.gyro_z,
                                                            dev->config.gyro_fs);

    /* 转换温度数据 */
    sensor_data->temperature = icm42688_convert_temp_raw_to_celsius(raw_data.temperature);

    /* 更新时间戳 */
    if (dev->comm.get_tick)
    {
        sensor_data->timestamp = dev->comm.get_tick();
    }
    else
    {
        sensor_data->timestamp = 0;
    }

    return ICM42688_OK;
}
/* ========================================================================== */
/*                          FIFO相关函数实现                                */
/* ========================================================================== */

/**
 * @brief 获取FIFO数据包数量
 * @param dev 设备句柄指针
 * @param packet_count 数据包数量输出
 * @return 错误码
 */
icm42688_err_t icm42688_get_fifo_packet_count(icm42688_device_t *dev,
                                              uint16_t *packet_count)
{
    ICM42688_CHECK_INIT(dev);
    ICM42688_CHECK_PARAM(packet_count);

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 读取FIFO计数器 */
    uint8_t fifo_count[2];
    icm42688_err_t ret = icm42688_read_reg_internal(dev, ICM42688_REG_FIFO_COUNTH,
                                                    fifo_count, 2);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    /* 计算数据包数量 */
    uint16_t fifo_bytes = (fifo_count[0] << 8) | fifo_count[1];
    uint8_t packet_size = 12; /* 基础数据包大小 */

    if (dev->config.fifo_accel_en && dev->config.fifo_gyro_en && dev->config.fifo_temp_en)
    {
        packet_size = 14; /* 加速度+陀螺仪+温度 */
    }
    else if (dev->config.fifo_accel_en && dev->config.fifo_gyro_en)
    {
        packet_size = 12; /* 加速度+陀螺仪 */
    }
    else if (dev->config.fifo_accel_en)
    {
        packet_size = 6; /* 仅加速度 */
    }
    else if (dev->config.fifo_gyro_en)
    {
        packet_size = 6; /* 仅陀螺仪 */
    }

    *packet_count = fifo_bytes / packet_size;

    return ICM42688_OK;
}

/**
 * @brief 读取FIFO数据
 * @param dev 设备句柄指针
 * @param packets 数据包缓冲区
 * @param max_packets 最大数据包数
 * @param read_packets 实际读取的数据包数
 * @return 错误码
 */
icm42688_err_t icm42688_read_fifo(icm42688_device_t *dev,
                                  icm42688_fifo_packet_t *packets,
                                  uint16_t max_packets,
                                  uint16_t *read_packets)
{
    ICM42688_CHECK_INIT(dev);
    ICM42688_CHECK_PARAM(packets);
    ICM42688_CHECK_PARAM(read_packets);

    /* 获取当前FIFO数据包数量 */
    uint16_t packet_count;
    icm42688_err_t ret = icm42688_get_fifo_packet_count(dev, &packet_count);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    /* 限制读取数量 */
    *read_packets = (packet_count > max_packets) ? max_packets : packet_count;
    if (*read_packets == 0)
    {
        return ICM42688_OK;
    }

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 计算数据包大小 */
    uint8_t packet_size = 12; /* 基础数据包大小 */
    if (dev->config.fifo_accel_en && dev->config.fifo_gyro_en && dev->config.fifo_temp_en)
    {
        packet_size = 14; /* 加速度+陀螺仪+温度 */
    }
    else if (dev->config.fifo_accel_en && dev->config.fifo_gyro_en)
    {
        packet_size = 12; /* 加速度+陀螺仪 */
    }
    else if (dev->config.fifo_accel_en)
    {
        packet_size = 6; /* 仅加速度 */
    }
    else if (dev->config.fifo_gyro_en)
    {
        packet_size = 6; /* 仅陀螺仪 */
    }

    /* 读取FIFO数据 */
    for (uint16_t i = 0; i < *read_packets; i++)
    {
        uint8_t fifo_data[14]; /* 最大数据包大小 */

        ret = icm42688_read_reg_internal(dev, ICM42688_REG_FIFO_DATA, fifo_data, packet_size);
        if (ret != ICM42688_OK)
        {
            *read_packets = i; /* 更新实际读取数量 */
            return ret;
        }

        /* 解析数据包头 */
        packets[i].header = fifo_data[0];

        /* 解析加速度数据 */
        if (dev->config.fifo_accel_en)
        {
            packets[i].accel[0] = (int16_t)((fifo_data[1] << 8) | fifo_data[2]);
            packets[i].accel[1] = (int16_t)((fifo_data[3] << 8) | fifo_data[4]);
            packets[i].accel[2] = (int16_t)((fifo_data[5] << 8) | fifo_data[6]);
        }

        /* 解析陀螺仪数据 */
        if (dev->config.fifo_gyro_en)
        {
            int offset = dev->config.fifo_accel_en ? 7 : 1;
            packets[i].gyro[0] = (int16_t)((fifo_data[offset] << 8) | fifo_data[offset + 1]);
            packets[i].gyro[1] = (int16_t)((fifo_data[offset + 2] << 8) | fifo_data[offset + 3]);
            packets[i].gyro[2] = (int16_t)((fifo_data[offset + 4] << 8) | fifo_data[offset + 5]);
        }

        /* 解析温度数据 */
        if (dev->config.fifo_temp_en)
        {
            int offset = (dev->config.fifo_accel_en ? 7 : 1) + (dev->config.fifo_gyro_en ? 6 : 0);
            packets[i].temperature = fifo_data[offset];
        }

        /* 更新时间戳 */
        if (dev->comm.get_tick)
        {
            packets[i].timestamp = dev->comm.get_tick();
        }
        else
        {
            packets[i].timestamp = 0;
        }

        packets[i].packet_size = packet_size;
    }

    return ICM42688_OK;
}

/**
 * @brief 清空FIFO
 * @param dev 设备句柄指针
 * @return 错误码
 */
icm42688_err_t icm42688_flush_fifo(icm42688_device_t *dev)
{
    ICM42688_CHECK_INIT(dev);

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 读取所有FIFO数据直到为空 */
    uint16_t packet_count;
    icm42688_err_t ret;

    do
    {
        ret = icm42688_get_fifo_packet_count(dev, &packet_count);
        if (ret != ICM42688_OK)
        {
            return ret;
        }

        if (packet_count > 0)
        {
            /* 读取并丢弃数据 */
            uint8_t dummy_data;
            for (uint16_t i = 0; i < packet_count; i++)
            {
                ret = icm42688_read_reg_internal(dev, ICM42688_REG_FIFO_DATA, &dummy_data, 1);
                if (ret != ICM42688_OK)
                {
                    return ret;
                }
            }
        }
    } while (packet_count > 0);

    return ICM42688_OK;
}

/* ========================================================================== */
/*                          中断处理函数实现                                */
/* ========================================================================== */

/**
 * @brief 配置中断引脚
 * @param dev 设备句柄指针
 * @param int_num 中断号 (1或2)
 * @param mode 中断模式
 * @param drive 驱动方式
 * @param polarity 极性
 * @return 错误码
 */
icm42688_err_t icm42688_config_interrupt_pin(icm42688_device_t *dev,
                                             uint8_t int_num,
                                             icm42688_int_mode_t mode,
                                             icm42688_int_drive_t drive,
                                             icm42688_int_polarity_t polarity)
{
    ICM42688_CHECK_INIT(dev);

    if (int_num != 1 && int_num != 2)
    {
        return ICM42688_ERR_INVALID_PARAM;
    }

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 读取当前中断配置 */
    icm42688_reg_int_config0_t int_config;
    icm42688_err_t ret = icm42688_read_reg_internal(dev, ICM42688_REG_INT_CONFIG0,
                                                    &int_config.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    /* 更新配置 */
    if (int_num == 1)
    {
    }
    else
    {
    }

    /* 写回配置 */
    return icm42688_write_reg_internal(dev, ICM42688_REG_INT_CONFIG0,
                                       &int_config.reg, 1);
}

/**
 * @brief 使能中断源
 * @param dev 设备句柄指针
 * @param int_num 中断号 (1或2)
 * @param int_source 中断源掩码
 * @return 错误码
 */
icm42688_err_t icm42688_enable_interrupt_source(icm42688_device_t *dev,
                                                uint8_t int_num,
                                                uint8_t int_source)
{
    ICM42688_CHECK_INIT(dev);

    if (int_num != 1 && int_num != 2)
    {
        return ICM42688_ERR_INVALID_PARAM;
    }

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 确定中断源寄存器地址 */
    uint8_t int_source_reg;
    if (int_num == 1)
    {
        int_source_reg = ICM42688_REG_INT_SOURCE1;
    }
    else
    {
        int_source_reg = ICM42688_REG_INT_SOURCE4;
    }

    /* 读取当前中断源配置 */
    uint8_t current_source;
    icm42688_err_t ret = icm42688_read_reg_internal(dev, int_source_reg,
                                                    &current_source, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    /* 更新中断源 */
    current_source |= int_source;

    /* 写回配置 */
    return icm42688_write_reg_internal(dev, int_source_reg, &current_source, 1);
}

/**
 * @brief 读取中断状态
 * @param dev 设备句柄指针
 * @param status 中断状态输出
 * @return 错误码
 */
icm42688_err_t icm42688_read_interrupt_status(icm42688_device_t *dev,
                                              uint8_t *status)
{
    ICM42688_CHECK_INIT(dev);
    ICM42688_CHECK_PARAM(status);

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 读取中断状态寄存器 */
    return icm42688_read_reg_internal(dev, ICM42688_REG_INT_STATUS, status, 1);
}
/* ========================================================================== */
/*                          校准和自检函数实现                              */
/* ========================================================================== */

/**
 * @brief 执行传感器自检
 * @param dev 设备句柄指针
 * @param gyro_self_test 陀螺仪自检结果
 * @param accel_self_test 加速度计自检结果
 * @return 错误码
 */
icm42688_err_t icm42688_self_test(icm42688_device_t *dev,
                                  bool *gyro_self_test,
                                  bool *accel_self_test)
{
    ICM42688_CHECK_INIT(dev);
    ICM42688_CHECK_PARAM(gyro_self_test);
    ICM42688_CHECK_PARAM(accel_self_test);

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 保存当前配置 */
    icm42688_sensor_config_t original_config = dev->config;

    /* 配置自检参数 */
    dev->config.gyro_mode = ICM42688_MODE_LOW_NOISE;
    dev->config.accel_mode = ICM42688_MODE_LOW_NOISE;
    dev->config.gyro_odr = ICM42688_ODR_200HZ;
    dev->config.accel_odr = ICM42688_ODR_200HZ;

    icm42688_err_t ret = icm42688_config_sensor(dev, &dev->config);
    if (ret != ICM42688_OK)
    {
        goto restore_config;
    }

    /* 读取自检前的数据 */
    icm42688_raw_data_t pre_test_data;
    ret = icm42688_read_raw_data(dev, &pre_test_data);
    if (ret != ICM42688_OK)
    {
        goto restore_config;
    }

    /* 启动自检 */
    icm42688_reg_self_test_config_t self_test_config = {
        .bits.EN_GX_ST = 1,
        .bits.EN_GY_ST = 1,
        .bits.EN_GZ_ST = 1,
        .bits.EN_AX_ST = 1,
        .bits.EN_AY_ST = 1,
        .bits.EN_AZ_ST = 1,
        .bits.ACCEL_ST_POWER = 1};

    ret = icm42688_write_reg_internal(dev, ICM42688_REG_SELF_TEST_CONFIG,
                                      &self_test_config.reg, 1);
    if (ret != ICM42688_OK)
    {
        goto restore_config;
    }

    /* 等待自检完成 */
    if (dev->comm.delay_ms)
    {
        dev->comm.delay_ms(100);
    }

    /* 读取自检后的数据 */
    icm42688_raw_data_t post_test_data;
    ret = icm42688_read_raw_data(dev, &post_test_data);
    if (ret != ICM42688_OK)
    {
        goto restore_config;
    }

    /* 停止自检 */
    self_test_config.reg = 0;
    ret = icm42688_write_reg_internal(dev, ICM42688_REG_SELF_TEST_CONFIG,
                                      &self_test_config.reg, 1);
    if (ret != ICM42688_OK)
    {
        goto restore_config;
    }

    /* 计算自检响应 */
    int16_t gyro_response[3], accel_response[3];

    gyro_response[0] = post_test_data.gyro_x - pre_test_data.gyro_x;
    gyro_response[1] = post_test_data.gyro_y - pre_test_data.gyro_y;
    gyro_response[2] = post_test_data.gyro_z - pre_test_data.gyro_z;

    accel_response[0] = post_test_data.accel_x - pre_test_data.accel_x;
    accel_response[1] = post_test_data.accel_y - pre_test_data.accel_y;
    accel_response[2] = post_test_data.accel_z - pre_test_data.accel_z;
    /* 检查自检结果（根据数据手册的阈值） */
    *gyro_self_test = true;
    *accel_self_test = true;

    /* 陀螺仪自检阈值检查 */
    for (int i = 0; i < 3; i++)
    {
        if (abs(gyro_response[i]) < ICM42688_GYRO_SELF_TEST_MIN ||
            abs(gyro_response[i]) > ICM42688_GYRO_SELF_TEST_MAX)
        {
            *gyro_self_test = false;
            break;
        }
    }

    /* 加速度计自检阈值检查 */
    for (int i = 0; i < 3; i++)
    {
        if (abs(accel_response[i]) < ICM42688_ACCEL_SELF_TEST_MIN ||
            abs(accel_response[i]) > ICM42688_ACCEL_SELF_TEST_MAX)
        {
            *accel_self_test = false;
            break;
        }
    }

restore_config:
    /* 恢复原始配置 */
    dev->config = original_config;
    icm42688_config_sensor(dev, &dev->config);

    return ret;
}

/**
 * @brief 校准传感器
 * @param dev 设备句柄指针
 * @param samples 采样数量
 * @param timeout_ms 超时时间
 * @return 错误码
 */
icm42688_err_t icm42688_calibrate_sensor(icm42688_device_t *dev,
                                         uint32_t samples,
                                         uint32_t timeout_ms)
{
    ICM42688_CHECK_INIT(dev);

    if (samples == 0)
    {
        return ICM42688_ERR_INVALID_PARAM;
    }

    /* 保存当前配置 */
    icm42688_sensor_config_t original_config = dev->config;

    /* 配置校准参数 */
    dev->config.gyro_mode = ICM42688_MODE_LOW_NOISE;
    dev->config.accel_mode = ICM42688_MODE_LOW_NOISE;
    dev->config.gyro_odr = ICM42688_ODR_100HZ;
    dev->config.accel_odr = ICM42688_ODR_100HZ;

    icm42688_err_t ret = icm42688_config_sensor(dev, &dev->config);
    if (ret != ICM42688_OK)
    {
        goto restore_config;
    }

    /* 初始化校准数据 */
    int32_t gyro_sum[3] = {0};
    int32_t accel_sum[3] = {0};

    uint32_t start_time = 0;
    if (dev->comm.get_tick)
    {
        start_time = dev->comm.get_tick();
    }

    /* 采集校准数据 */
    for (uint32_t i = 0; i < samples; i++)
    {
        /* 检查超时 */
        if (dev->comm.get_tick &&
            (dev->comm.get_tick() - start_time > timeout_ms))
        {
            ret = ICM42688_ERR_TIMEOUT;
            goto restore_config;
        }

        /* 读取数据 */
        icm42688_raw_data_t raw_data;
        ret = icm42688_read_raw_data(dev, &raw_data);
        if (ret != ICM42688_OK)
        {
            goto restore_config;
        }

        /* 累加数据 */
        gyro_sum[0] += raw_data.gyro_x;
        gyro_sum[1] += raw_data.gyro_y;
        gyro_sum[2] += raw_data.gyro_z;

        accel_sum[0] += raw_data.accel_x;
        accel_sum[1] += raw_data.accel_y;
        accel_sum[2] += raw_data.accel_z;

        /* 延时 */
        if (dev->comm.delay_ms)
        {
            dev->comm.delay_ms(10);
        }
    }

    /* 计算平均值作为偏移量 */
    dev->calib.gyro_offset[0] = (float)gyro_sum[0] / samples;
    dev->calib.gyro_offset[1] = (float)gyro_sum[1] / samples;
    dev->calib.gyro_offset[2] = (float)gyro_sum[2] / samples;

    dev->calib.accel_offset[0] = (float)accel_sum[0] / samples;
    dev->calib.accel_offset[1] = (float)accel_sum[1] / samples;
    dev->calib.accel_offset[2] = (float)accel_sum[2] / samples;

    /* 设置校准标志 */
    dev->calib.calibrated = true;

restore_config:
    /* 恢复原始配置 */
    dev->config = original_config;
    icm42688_config_sensor(dev, &dev->config);

    return ret;
}

/**
 * @brief 设置用户偏移量
 * @param dev 设备句柄指针
 * @param axis 轴选择 (0:X, 1:Y, 2:Z)
 * @param gyro_offset 陀螺仪偏移量
 * @param accel_offset 加速度计偏移量
 * @return 错误码
 */
icm42688_err_t icm42688_set_user_offset(icm42688_device_t *dev,
                                        uint8_t axis,
                                        int16_t gyro_offset,
                                        int16_t accel_offset)
{
    ICM42688_CHECK_INIT(dev);

    if (axis > 2)
    {
        return ICM42688_ERR_INVALID_PARAM;
    }

    ICM42688_BANK_SELECT(dev, ICM42688_BANK4);

    /* 设置陀螺仪偏移量 */
    uint8_t gyro_offset_reg = ICM42688_REG_OFFSET_USER0 + axis * 2;
    uint8_t gyro_data[2] = {
        (uint8_t)((gyro_offset >> 8) & 0xFF),
        (uint8_t)(gyro_offset & 0xFF)};

    icm42688_err_t ret = icm42688_write_reg_internal(dev, gyro_offset_reg,
                                                     gyro_data, 2);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    /* 设置加速度计偏移量 */
    uint8_t accel_offset_reg = ICM42688_REG_OFFSET_USER6 + axis * 2;
    uint8_t accel_data[2] = {
        (uint8_t)((accel_offset >> 8) & 0xFF),
        (uint8_t)(accel_offset & 0xFF)};

    return icm42688_write_reg_internal(dev, accel_offset_reg, accel_data, 2);
}

/* ========================================================================== */
/*                          工具和状态函数实现                              */
/* ========================================================================== */

/**
 * @brief 读取设备状态
 * @param dev 设备句柄指针
 * @param status 设备状态输出
 * @return 错误码
 */
icm42688_err_t icm42688_get_device_status(icm42688_device_t *dev,
                                          icm42688_device_status_t *status)
{
    ICM42688_CHECK_INIT(dev);
    ICM42688_CHECK_PARAM(status);

    /* 复制设备状态 */
    *status = dev->status;

    /* 读取中断状态寄存器 */
    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    uint8_t int_status;
    icm42688_err_t ret = icm42688_read_reg_internal(dev, ICM42688_REG_INT_STATUS,
                                                    &int_status, 1);
    if (ret == ICM42688_OK)
    {
        status->data_ready = (int_status & ICM42688_INT_SOURCE_DATA_READY) != 0;
        status->fifo_watermark = (int_status & ICM42688_INT_SOURCE_FIFO_THS) != 0;
        status->fifo_overflow = (int_status & ICM42688_INT_SOURCE_FIFO_OVF) != 0;
    }

    /* 读取FIFO计数器 */
    uint16_t fifo_count;
    ret = icm42688_get_fifo_packet_count(dev, &fifo_count);
    if (ret == ICM42688_OK)
    {
        status->fifo_count = fifo_count;
    }

    return ICM42688_OK;
}

/**
 * @brief 读取芯片温度
 * @param dev 设备句柄指针
 * @param temperature 温度输出 [°C]
 * @return 错误码
 */
icm42688_err_t icm42688_read_temperature(icm42688_device_t *dev,
                                         float *temperature)
{
    ICM42688_CHECK_INIT(dev);
    ICM42688_CHECK_PARAM(temperature);

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 读取温度数据 */
    uint8_t temp_data[2];
    icm42688_err_t ret = icm42688_read_reg_internal(dev, ICM42688_REG_TEMP_DATA1,
                                                    temp_data, 2);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    /* 转换温度数据 */
    int16_t raw_temp = (int16_t)((temp_data[0] << 8) | temp_data[1]);
    *temperature = icm42688_convert_temp_raw_to_celsius(raw_temp);

    return ICM42688_OK;
}

/**
 * @brief 获取设备信息
 * @param dev 设备句柄指针
 * @param info 设备信息字符串缓冲区
 * @param len 缓冲区长度
 * @return 错误码
 */
icm42688_err_t icm42688_get_device_info(icm42688_device_t *dev,
                                        char *info, uint32_t len)
{
    ICM42688_CHECK_INIT(dev);
    ICM42688_CHECK_PARAM(info);

    if (len < 100)
    {
        return ICM42688_ERR_INVALID_PARAM;
    }

    /* 生成设备信息字符串 */
    snprintf(info, len,
             "ICM-42688-P Motion Sensor\n"
             "Device ID: 0x%02X\n"
             "Initialized: %s\n"
             "Gyro Mode: %s\n"
             "Accel Mode: %s\n"
             "Sample Count: %" PRIu32 "\n"
             "Calibrated: %s",
             dev->whoami,
             dev->initialized ? "Yes" : "No",
             dev->config.gyro_mode == ICM42688_MODE_OFF ? "Off" : dev->config.gyro_mode == ICM42688_MODE_LOW_NOISE ? "Low Noise"
                                                                                                                   : "Low Power",
             dev->config.accel_mode == ICM42688_MODE_OFF ? "Off" : dev->config.accel_mode == ICM42688_MODE_LOW_NOISE ? "Low Noise"
                                                                                                                     : "Low Power",
             dev->sample_count,
             dev->calib.calibrated ? "Yes" : "No");

    return ICM42688_OK;
}

/**
 * @brief 转换原始加速度数据到物理量
 * @param raw_data 原始数据
 * @param fs 满量程范围
 * @param accel 加速度输出 [m/s²]
 */
void icm42688_convert_accel_data(int16_t raw_data, icm42688_accel_fs_t fs, float *accel)
{
    *accel = icm42688_convert_accel_raw_to_ms2(raw_data, fs);
}

/**
 * @brief 转换原始陀螺仪数据到物理量
 * @param raw_data 原始数据
 * @param fs 满量程范围
 * @param gyro 角速度输出 [rad/s]
 */
void icm42688_convert_gyro_data(int16_t raw_data, icm42688_gyro_fs_t fs, float *gyro)
{
    *gyro = icm42688_convert_gyro_raw_to_rads(raw_data, fs);
}

/**
 * @brief 获取错误码描述字符串
 * @param err 错误码
 * @return 错误描述字符串
 */
const char *icm42688_get_error_string(icm42688_err_t err)
{
    switch (err)
    {
    case ICM42688_OK:
        return "Operation successful";
    case ICM42688_ERR_COMM:
        return "Communication error";
    case ICM42688_ERR_NOT_READY:
        return "Device not ready";
    case ICM42688_ERR_INVALID_PARAM:
        return "Invalid parameter";
    case ICM42688_ERR_TIMEOUT:
        return "Operation timeout";
    case ICM42688_ERR_FIFO_OVERFLOW:
        return "FIFO overflow";
    case ICM42688_ERR_SELF_TEST_FAIL:
        return "Self-test failed";
    case ICM42688_ERR_NOT_INITIALIZED:
        return "Device not initialized";
    default:
        return "Unknown error";
    }
}

/* ========================================================================== */
/*                          低功耗模式函数实现                              */
/* ========================================================================== */

/**
 * @brief 进入低功耗模式
 * @param dev 设备句柄指针
 * @return 错误码
 */
icm42688_err_t icm42688_enter_low_power_mode(icm42688_device_t *dev)
{
    ICM42688_CHECK_INIT(dev);

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 配置为低功耗模式 */
    icm42688_reg_pwr_mgmt0_t pwr_mgmt0;
    icm42688_err_t ret = icm42688_read_reg_internal(dev, ICM42688_REG_PWR_MGMT0,
                                                    &pwr_mgmt0.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    pwr_mgmt0.bits.GYRO_MODE = ICM42688_MODE_LOW_POWER;
    pwr_mgmt0.bits.ACCEL_MODE = ICM42688_MODE_LOW_POWER;

    return icm42688_write_reg_internal(dev, ICM42688_REG_PWR_MGMT0,
                                       &pwr_mgmt0.reg, 1);
}

/**
 * @brief 唤醒设备
 * @param dev 设备句柄指针
 * @return 错误码
 */
icm42688_err_t icm42688_wakeup(icm42688_device_t *dev)
{
    ICM42688_CHECK_INIT(dev);

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 配置为低噪声模式 */
    icm42688_reg_pwr_mgmt0_t pwr_mgmt0;
    icm42688_err_t ret = icm42688_read_reg_internal(dev, ICM42688_REG_PWR_MGMT0,
                                                    &pwr_mgmt0.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    pwr_mgmt0.bits.GYRO_MODE = ICM42688_MODE_LOW_NOISE;
    pwr_mgmt0.bits.ACCEL_MODE = ICM42688_MODE_LOW_NOISE;

    ret = icm42688_write_reg_internal(dev, ICM42688_REG_PWR_MGMT0,
                                      &pwr_mgmt0.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    /* 等待设备稳定 */
    if (dev->comm.delay_ms)
    {
        dev->comm.delay_ms(ICM42688_WAKEUP_DELAY_MS);
    }

    return ICM42688_OK;
}

/* ========================================================================== */
/*                          外部时钟配置函数实现                            */
/* ========================================================================== */

/**
 * @brief 配置外部时钟输入
 * @param dev 设备句柄指针
 * @param enable 使能标志
 * @param frequency 外部时钟频率 (Hz)
 * @return 错误码
 */
icm42688_err_t icm42688_config_external_clock(icm42688_device_t *dev,
                                              bool enable,
                                              uint32_t frequency)
{
    ICM42688_CHECK_INIT(dev);

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 配置RTC模式 */
    icm42688_reg_intf_config1_t intf_config1;
    icm42688_err_t ret = icm42688_read_reg_internal(dev, ICM42688_REG_INTF_CONFIG1,
                                                    &intf_config1.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    intf_config1.bits.RTC_MOD = enable ? 1 : 0;

    ret = icm42688_write_reg_internal(dev, ICM42688_REG_INTF_CONFIG1,
                                      &intf_config1.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    if (enable)
    {
        /* 配置外部时钟频率 */
        ICM42688_BANK_SELECT(dev, ICM42688_BANK1);

        /* 根据频率配置相关寄存器 */
        /* 具体配置根据数据手册实现 */
    }

    return ICM42688_OK;
}

/* ========================================================================== */
/*                          滤波器配置函数实现                              */
/* ========================================================================== */

/**
 * @brief 配置陀螺仪滤波器
 * @param dev 设备句柄指针
 * @param order 滤波器阶数
 * @param bandwidth 滤波器带宽
 * @return 错误码
 */
icm42688_err_t icm42688_config_gyro_filter(icm42688_device_t *dev,
                                           icm42688_filter_order_t order,
                                           icm42688_filter_bw_t bandwidth)
{
    ICM42688_CHECK_INIT(dev);

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 配置陀螺仪UI滤波器 */
    icm42688_reg_gyro_config1_t gyro_config1;
    icm42688_err_t ret = icm42688_read_reg_internal(dev, ICM42688_REG_GYRO_CONFIG1,
                                                    &gyro_config1.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    gyro_config1.bits.GYRO_UI_FILT_ORD = order;
    gyro_config1.bits.GYRO_DEC2_M2_ORD = bandwidth;

    return icm42688_write_reg_internal(dev, ICM42688_REG_GYRO_CONFIG1,
                                       &gyro_config1.reg, 1);
}

/**
 * @brief 配置加速度计滤波器
 * @param dev 设备句柄指针
 * @param order 滤波器阶数
 * @param bandwidth 滤波器带宽
 * @return 错误码
 */
icm42688_err_t icm42688_config_accel_filter(icm42688_device_t *dev,
                                            icm42688_filter_order_t order,
                                            icm42688_filter_bw_t bandwidth)
{
    ICM42688_CHECK_INIT(dev);

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 配置加速度计UI滤波器 */
    icm42688_reg_accel_config1_t accel_config1;
    icm42688_err_t ret = icm42688_read_reg_internal(dev, ICM42688_REG_ACCEL_CONFIG1,
                                                    &accel_config1.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    accel_config1.bits.ACCEL_UI_FILT_ORD = order;
    accel_config1.bits.ACCEL_DEC2_M2_ORD = bandwidth;

    return icm42688_write_reg_internal(dev, ICM42688_REG_ACCEL_CONFIG1,
                                       &accel_config1.reg, 1);
}

/* ========================================================================== */
/*                          时间戳功能函数实现                              */
/* ========================================================================== */

/**
 * @brief 读取时间戳
 * @param dev 设备句柄指针
 * @param timestamp 时间戳输出
 * @return 错误码
 */
icm42688_err_t icm42688_read_timestamp(icm42688_device_t *dev,
                                       uint32_t *timestamp)
{
    ICM42688_CHECK_INIT(dev);
    ICM42688_CHECK_PARAM(timestamp);

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 读取时间戳寄存器 */
    uint8_t timestamp_data[3];
    icm42688_err_t ret = icm42688_read_reg_internal(dev, ICM42688_REG_TMST_FSYNCH,
                                                    timestamp_data, 3);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    /* 组合时间戳值 */
    *timestamp = (timestamp_data[0] << 16) | (timestamp_data[1] << 8) | timestamp_data[2];

    return ICM42688_OK;
}

/**
 * @brief 配置时间戳
 * @param dev 设备句柄指针
 * @param enable 使能标志
 * @param source 时间戳源
 * @return 错误码
 */
icm42688_err_t icm42688_config_timestamp(icm42688_device_t *dev,
                                         bool enable,
                                         icm42688_timestamp_source_t source)
{
    ICM42688_CHECK_INIT(dev);

    ICM42688_BANK_SELECT(dev, ICM42688_BANK0);

    /* 配置时间戳寄存器 */
    icm42688_reg_tmst_config_t tmst_config;
    icm42688_err_t ret = icm42688_read_reg_internal(dev, ICM42688_REG_TMST_CONFIG,
                                                    &tmst_config.reg, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    tmst_config.bits.TMST_EN = enable ? 1 : 0;
    tmst_config.bits.TMST_FSYNC_EN = source;

    return icm42688_write_reg_internal(dev, ICM42688_REG_TMST_CONFIG,
                                       &tmst_config.reg, 1);
}

/* ========================================================================== */
/*                          内部工具函数实现                               */
/* ========================================================================== */

/**
 * @brief 计算校验和
 * @param data 数据缓冲区
 * @param len 数据长度
 * @return 校验和
 */
static uint8_t icm42688_calculate_checksum(const uint8_t *data, uint16_t len)
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        checksum += data[i];
    }
    return checksum;
}

/**
 * @brief 验证寄存器配置
 * @param dev 设备句柄指针
 * @param reg_addr 寄存器地址
 * @param expected 期望值
 * @param mask 掩码
 * @return 错误码
 */
static icm42688_err_t icm42688_verify_register(icm42688_device_t *dev,
                                               uint8_t reg_addr,
                                               uint8_t expected,
                                               uint8_t mask)
{
    uint8_t actual;
    icm42688_err_t ret = icm42688_read_reg_internal(dev, reg_addr, &actual, 1);
    if (ret != ICM42688_OK)
    {
        return ret;
    }

    if ((actual & mask) != (expected & mask))
    {
        return ICM42688_ERR_COMM;
    }

    return ICM42688_OK;
}
