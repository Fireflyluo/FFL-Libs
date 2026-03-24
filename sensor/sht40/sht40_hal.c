/**
 * @file sht40_hal.c
 * @brief SHT40 温湿度传感器硬件抽象层驱动
 * 
 * 本驱动使用弱函数定义，用户需要在自己的项目中实现以下函数：
 * - SHT40_I2C_Write: I2C 写入函数
 * - SHT40_I2C_Read: I2C 读取函数
 * - SHT40_Delay: 延时函数（毫秒）
 */

#include "sht40_hal.h"
#include <stdio.h>

#if defined(_MSC_VER)
#define SHT40_WEAK
#elif defined(__GNUC__) || defined(__clang__)
#define SHT40_WEAK __attribute__((weak))
#else
#define SHT40_WEAK
#endif

/* 弱函数定义 - 用户需要在自己的项目中实现 */

/**
 * @brief I2C 写入函数（弱定义）
 * @param i2c_num I2C 总线编号
 * @param addr 设备地址
 * @param data 数据缓冲区
 * @param len 数据长度
 * @return 0-成功，非 0-失败
 */
#ifndef SHT40_DISABLE_DEFAULT_STUBS
SHT40_WEAK
uint8_t SHT40_I2C_Write(uint8_t i2c_num, uint8_t addr, uint8_t *data, uint8_t len)
{
    (void)i2c_num;
    (void)addr;
    (void)data;
    (void)len;
    return 1; // 返回错误，提示用户需要实现此函数
}

/**
 * @brief I2C 读取函数（弱定义）
 * @param i2c_num I2C 总线编号
 * @param addr 设备地址
 * @param data 数据缓冲区
 * @param len 数据长度
 * @return 0-成功，非 0-失败
 */
SHT40_WEAK
uint8_t SHT40_I2C_Read(uint8_t i2c_num, uint8_t addr, uint8_t *data, uint8_t len)
{
    (void)i2c_num;
    (void)addr;
    (void)data;
    (void)len;
    return 1; // 返回错误，提示用户需要实现此函数
}

/**
 * @brief 延时函数（弱定义）
 * @param ms 延时毫秒数
 */
SHT40_WEAK
void SHT40_Delay(uint32_t ms)
{
    (void)ms;
    // 空实现，用户需要实现自己的延时函数
}
#endif
/*************************************************************************************************
 *	函 数 名: SHT40_Init
 *	入口参数：无
 *	返回值：无
 *	函数功能：初始化 SHT40 使用的 I2C 总线
 *************************************************************************************************/
void SHT40_Init(void)
{
    uint32_t serial = 0;
    // 执行软件复位
    SHT40_Soft_Reset();
    SHT40_Delay(10); // 复位后需要等待
    serial = SHT40_Read_Serial_Number();
    printf("SHT40 Serial Number: 0x%08lX\r\n", serial);
}

/*************************************************************************************************
 *	函 数 名: SHT40_Soft_Reset
 *	入口参数：无
 *	返回值：0-成功，非 0-失败
 *	函数功能：软件复位 SHT40
 *************************************************************************************************/
uint8_t SHT40_Soft_Reset(void)
{
    uint8_t cmd = SHT40_SOFT_RESET;

    // 发送复位命令
    uint8_t ret = SHT40_I2C_Write(SHT40_I2C_NUM, SHT40_I2C_ADDR, &cmd, 1);
    if (ret != 0)
    {
        return 1; // 发送失败
    }

    SHT40_Delay(2); // 等待复位完成

    return 0; // 成功
}

/*************************************************************************************************
 *	函 数 名: SHT40_Read_Temperature_Humidity
 *	入口参数：temperature - 温度指针，humidity - 湿度指针
 *	返回值：0-成功，非 0-错误代码
 *	函数功能：以高精度读取温度和湿度
 *	说    明：使用默认的高精度测量命令
 *************************************************************************************************/
uint8_t SHT40_Read_Temperature_Humidity(float *temperature, float *humidity)
{
    return SHT40_Read_Temperature_Humidity_Ex(SHT40_MEASURE_TEMPERATURE_HUMIDITY_HIGH_PRECISION, temperature, humidity);
}

/*************************************************************************************************
 *	函 数 名: SHT40_Read_Temperature_Humidity_Ex
 *	入口参数：cmd - 测量命令，temperature - 温度指针，humidity - 湿度指针
 *	返回值：0-成功，非 0-错误代码
 *	函数功能：读取温度和湿度
 *	说    明：支持不同精度的测量模式
 *************************************************************************************************/
uint8_t SHT40_Read_Temperature_Humidity_Ex(uint8_t cmd, float *temperature, float *humidity)
{
    uint8_t rx_data[6] = {0};
    uint8_t ret        = 0;

    // 1. 发送测量命令
    ret = SHT40_I2C_Write(SHT40_I2C_NUM, SHT40_I2C_ADDR, &cmd, 1);
    if (ret != 0)
    {
        return 1; // 发送命令失败
    }

    // 2. 等待测量完成
    // 高精度：8.2ms, 中精度：4.5ms, 低精度：1.7ms
    switch (cmd)
    {
    case SHT40_MEASURE_TEMPERATURE_HUMIDITY_HIGH_PRECISION:
        SHT40_Delay(10); // 等待 10ms 确保完成
        break;
    case SHT40_MEASURE_TEMPERATURE_HUMIDITY_MEDIUM_PRECISION:
        SHT40_Delay(6); // 等待 6ms
        break;
    case SHT40_MEASURE_TEMPERATURE_HUMIDITY_LOW_PRECISION:
        SHT40_Delay(3); // 等待 3ms
        break;
    default:
        SHT40_Delay(10); // 默认等待 10ms
        break;
    }

    // 3. 读取 6 字节数据
    ret = SHT40_I2C_Read(SHT40_I2C_NUM, SHT40_I2C_ADDR, rx_data, 6);
    if (ret != 0)
    {
        return 2; // 读取数据失败
    }

    // 4. 解析温湿度数据
    // 温度：前 2 字节，湿度：中间 2 字节，后 2 字节是 CRC 校验
    uint16_t temp_raw = (rx_data[0] << 8) | rx_data[1];
    uint16_t hum_raw  = (rx_data[3] << 8) | rx_data[4];

    // 5. 转换为实际值
    *temperature = -45.0f + 175.0f * (temp_raw / 65535.0f);
    *humidity    = -6.0f + 125.0f * (hum_raw / 65535.0f);

    // 限制湿度范围在 0-100%
    if (*humidity < 0.0f)
        *humidity = 0.0f;
    if (*humidity > 100.0f)
        *humidity = 100.0f;

    return 0; // 成功
}

/*************************************************************************************************
 *	函 数 名: SHT40_Read_Serial_Number
 *	入口参数：无
 *	返回值：32 位序列号
 *	函数功能：读取 SHT40 的出场唯一序列号
 *************************************************************************************************/
uint32_t SHT40_Read_Serial_Number(void)
{
    uint8_t cmd        = SHT40_READ_SERIAL_NUMBER;
    uint8_t rx_data[6] = {0};
    uint8_t ret        = 0;
    // 1. 发送读取序列号命令
    ret = SHT40_I2C_Write(SHT40_I2C_NUM, SHT40_I2C_ADDR, &cmd, 1);
    if (ret != 0)
    {
        return 0; // 发送命令失败
    }

    SHT40_Delay(1); // 短暂延时

    // 2. 读取序列号数据
    ret = SHT40_I2C_Read(SHT40_I2C_NUM, SHT40_I2C_ADDR, rx_data, 6);
    if (ret != 0)
    {
        return 0; // 读取数据失败
    }
    SHT40_Delay(1); // 短暂延时
    // 3. 组合为 32 位序列号
    // 数据格式：字节 0,1,3,4 是序列号，字节 2,5 是 CRC
    uint32_t serial =
        ((uint32_t)rx_data[0] << 24) | ((uint32_t)rx_data[1] << 16) | ((uint32_t)rx_data[3] << 8) | (rx_data[4]);

    return serial;
}

/*************************************************************************************************
 *	函 数 名: SHT40_Heater
 *	入口参数：heater_cmd - 加热器命令
 *	返回值：0-成功，非 0-失败
 *	函数功能：启动内部加热器
 *	说    明：加热时间不能超过运行时间的 10％，否则会过热
 *************************************************************************************************/
uint8_t SHT40_Heater(uint8_t heater_cmd)
{
    // 检查是否为有效的加热命令
    if (heater_cmd != SHT40_HEATER_200mW_1s && heater_cmd != SHT40_HEATER_200mW_100ms &&
        heater_cmd != SHT40_HEATER_110mW_1s && heater_cmd != SHT40_HEATER_110mW_100ms &&
        heater_cmd != SHT40_HEATER_20mW_1s && heater_cmd != SHT40_HEATER_20mW_100ms)
    {
        return 1; // 无效的加热命令
    }

    // 发送加热命令
    uint8_t ret = SHT40_I2C_Write(SHT40_I2C_NUM, SHT40_I2C_ADDR, &heater_cmd, 1);
    if (ret != 0)
    {
        return 2; // 发送失败
    }

    // 根据加热时长等待
    switch (heater_cmd)
    {
    case SHT40_HEATER_200mW_1s:
    case SHT40_HEATER_110mW_1s:
    case SHT40_HEATER_20mW_1s:
        SHT40_Delay(1200); // 等待 1.2 秒
        break;

    case SHT40_HEATER_200mW_100ms:
    case SHT40_HEATER_110mW_100ms:
    case SHT40_HEATER_20mW_100ms:
        SHT40_Delay(150); // 等待 150ms
        break;

    default:
        break;
    }

    return 0; // 成功
}