#ifndef __SHT40_H__
#define __SHT40_H__

#include "stdint.h"

// 设备地址定义
#define SHT40_ADDR_WRITE 0x88 // 写地址
#define SHT40_ADDR_READ  0x89 // 读地址
#define SHT40_I2C_ADDR   0x46 // 7 位地址

// I2C 总线编号（用户可以根据需要修改）
#ifndef SHT40_I2C_NUM
#define SHT40_I2C_NUM 0
#endif

// 命令定义
#define SHT40_MEASURE_TEMPERATURE_HUMIDITY                  0xFD
#define SHT40_MEASURE_TEMPERATURE_HUMIDITY_HIGH_PRECISION   0xFD
#define SHT40_MEASURE_TEMPERATURE_HUMIDITY_MEDIUM_PRECISION 0xF6
#define SHT40_MEASURE_TEMPERATURE_HUMIDITY_LOW_PRECISION    0xE0
#define SHT40_READ_SERIAL_NUMBER                            0x89
#define SHT40_SOFT_RESET                                    0x94
#define SHT40_HEATER_200mW_1s                               0x39
#define SHT40_HEATER_200mW_100ms                            0x32
#define SHT40_HEATER_110mW_1s                               0x2F
#define SHT40_HEATER_110mW_100ms                            0x24
#define SHT40_HEATER_20mW_1s                                0x1E
#define SHT40_HEATER_20mW_100ms                             0x15

// 弱函数声明 - 用户需要在自己的项目中实现
#ifdef __cplusplus
extern "C" {
#endif

// I2C 写入函数
uint8_t SHT40_I2C_Write(uint8_t i2c_num, uint8_t addr, uint8_t *data, uint8_t len);
// I2C 读取函数
uint8_t SHT40_I2C_Read(uint8_t i2c_num, uint8_t addr, uint8_t *data, uint8_t len);
// 延时函数
void SHT40_Delay(uint32_t ms);

// 函数声明
void SHT40_Init(void);
uint8_t SHT40_Read_Temperature_Humidity(float *temperature, float *humidity);
uint8_t SHT40_Read_Temperature_Humidity_Ex(uint8_t cmd, float *temperature, float *humidity);
uint32_t SHT40_Read_Serial_Number(void);
uint8_t SHT40_Soft_Reset(void);
uint8_t SHT40_Heater(uint8_t heater_cmd);

#ifdef __cplusplus
}
#endif

#endif