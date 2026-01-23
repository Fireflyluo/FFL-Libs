#include "sht40_hal.h"
#include "debug.h" // 用于调试输出
#include "board.h"

/*************************************************************************************************
 *	函 数 名: SHT40_Init
 *	入口参数: I2Cx - I2C外设指针
 *	返回值: 无
 *	函数功能: 初始化SHT40使用的I2C总线
 *************************************************************************************************/
void SHT40_Init(void)
{
    // 执行软件复位
    SHT40_Soft_Reset();
    SHT40_Delay(10); // 复位后需要等待
}

/*************************************************************************************************
 *	函 数 名: SHT40_Soft_Reset
 *	入口参数: 无
 *	返回值: 0-成功, 非0-失败
 *	函数功能: 软件复位SHT40
 *************************************************************************************************/
uint8_t SHT40_Soft_Reset(void)
{
    uint8_t cmd = SHT40_SOFT_RESET;
    uint8_t data[1];

    // 发送复位命令
    uint8_t ret = I2C_Master_Transmit_Polling(SHT40_I2C_ADDR, cmd, data, 0);
    if (ret != 0)
    {
        return 1; // 发送失败
    }

    SHT40_Delay(2); // 等待复位完成

    return 0; // 成功
}

/*************************************************************************************************
 *	函 数 名: SHT40_Read_Temperature_Humidity
 *	入口参数: temperature - 温度指针, humidity - 湿度指针
 *	返回值: 0-成功, 非0-错误代码
 *	函数功能: 以高精度读取温度和湿度
 *	说    明: 使用默认的高精度测量命令
 *************************************************************************************************/
uint8_t SHT40_Read_Temperature_Humidity(float *temperature, float *humidity)
{
    return SHT40_Read_Temperature_Humidity_Ex(SHT40_MEASURE_TEMPERATURE_HUMIDITY_HIGH_PRECISION, temperature, humidity);
}

/*************************************************************************************************
 *	函 数 名: SHT40_Read_Temperature_Humidity_Ex
 *	入口参数: cmd - 测量命令, temperature - 温度指针, humidity - 湿度指针
 *	返回值: 0-成功, 非0-错误代码
 *	函数功能: 读取温度和湿度
 *	说    明: 支持不同精度的测量模式
 *************************************************************************************************/
uint8_t SHT40_Read_Temperature_Humidity_Ex(uint8_t cmd, float *temperature, float *humidity)
{
    uint8_t rx_data[6] = {0};
    uint8_t ret = 0;

    // 1. 发送测量命令
    ret = I2C_Master_Transmit_Polling(SHT40_I2C_ADDR, cmd, NULL, 0);
    if (ret != 0)
    {
        return 1; // 发送命令失败
    }

    // 2. 等待测量完成
    // 高精度: 8.2ms, 中精度: 4.5ms, 低精度: 1.7ms
    switch (cmd)
    {
    case SHT40_MEASURE_TEMPERATURE_HUMIDITY_HIGH_PRECISION:
        SHT40_Delay(10); // 等待10ms确保完成
        break;
    case SHT40_MEASURE_TEMPERATURE_HUMIDITY_MEDIUM_PRECISION:
        SHT40_Delay(6); // 等待6ms
        break;
    case SHT40_MEASURE_TEMPERATURE_HUMIDITY_LOW_PRECISION:
        SHT40_Delay(3); // 等待3ms
        break;
    default:
        SHT40_Delay(10); // 默认等待10ms
        break;
    }

    // 3. 读取6字节数据
    ret = I2C_Master_Receive_Polling(SHT40_I2C_ADDR, 0xFF, rx_data, 6);
    if (ret != 0)
    {
        return 2; // 读取数据失败
    }

    // 4. 解析温湿度数据
    // 温度: 前2字节, 湿度: 中间2字节, 后2字节是CRC校验
    uint16_t temp_raw = (rx_data[0] << 8) | rx_data[1];
    uint16_t hum_raw = (rx_data[3] << 8) | rx_data[4];

    // 5. 转换为实际值
    *temperature = -45.0f + 175.0f * (temp_raw / 65535.0f);
    *humidity = -6.0f + 125.0f * (hum_raw / 65535.0f);

    // 限制湿度范围在0-100%
    if (*humidity < 0.0f)
        *humidity = 0.0f;
    if (*humidity > 100.0f)
        *humidity = 100.0f;

    return 0; // 成功
}

/*************************************************************************************************
 *	函 数 名: SHT40_Read_Serial_Number
 *	入口参数: 无
 *	返回值: 32位序列号
 *	函数功能: 读取SHT40的出场唯一序列号
 *************************************************************************************************/
uint32_t SHT40_Read_Serial_Number(void)
{
    uint8_t cmd = SHT40_READ_SERIAL_NUMBER;
    uint8_t rx_data[6] = {0};

    // 1. 发送读取序列号命令
    if (I2C_Master_Transmit_Polling(SHT40_I2C_ADDR, cmd, NULL, 0) != 0)
    {
        return 0xFFFFFFFF; // 发送失败
    }

    SHT40_Delay(1); // 短暂延时

    // 2. 读取序列号数据
    if (I2C_Master_Receive_Polling(SHT40_I2C_ADDR, 0xFF, rx_data, 6) != 0)
    {
        return 0xFFFFFFFF; // 读取失败
    }

    // 3. 组合为32位序列号
    // 数据格式: 字节0,1,3,4是序列号，字节2,5是CRC
    uint32_t serial = ((uint32_t)rx_data[0] << 24) |
                      ((uint32_t)rx_data[1] << 16) |
                      ((uint32_t)rx_data[3] << 8) |
                      (rx_data[4]);

    return serial;
}

/*************************************************************************************************
 *	函 数 名: SHT40_Heater
 *	入口参数: heater_cmd - 加热器命令
 *	返回值: 0-成功, 非0-失败
 *	函数功能: 启动内部加热器
 *	说    明: 加热时间不能超过运行时间的10％，否则会过热
 *************************************************************************************************/
uint8_t SHT40_Heater(uint8_t heater_cmd)
{
    // 检查是否为有效的加热命令
    if (heater_cmd != SHT40_HEATER_200mW_1s &&
        heater_cmd != SHT40_HEATER_200mW_100ms &&
        heater_cmd != SHT40_HEATER_110mW_1s &&
        heater_cmd != SHT40_HEATER_110mW_100ms &&
        heater_cmd != SHT40_HEATER_20mW_1s &&
        heater_cmd != SHT40_HEATER_20mW_100ms)
    {
        return 1; // 无效的加热命令
    }

    // 发送加热命令
    uint8_t ret = I2C_Master_Transmit_Polling(SHT40_I2C_ADDR, heater_cmd, NULL, 0);
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
        SHT40_Delay(1200); // 等待1.2秒
        break;

    case SHT40_HEATER_200mW_100ms:
    case SHT40_HEATER_110mW_100ms:
    case SHT40_HEATER_20mW_100ms:
        SHT40_Delay(150); // 等待150ms
        break;

    default:
        break;
    }

    return 0; // 成功
}

/*************************************************************************************************
 *	函 数 名: SHT40_Delay
 *	入口参数: ms - 延时毫秒数
 *	返回值: 无
 *	函数功能: 延时函数
 *************************************************************************************************/
void SHT40_Delay(uint32_t ms)
{
    // 调用系统延时函数
    Delay_Ms(ms);
}