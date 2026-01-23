#ifndef __SHT40_H__
#define __SHT40_H__

#include "stdint.h"

// 设备地址定义
#define SHT40_ADDR_WRITE 0x88      // 写地址
#define SHT40_ADDR_READ  0x89      // 读地址
#define SHT40_I2C_ADDR   0x46 << 1 // 7位地址

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

// 函数声明
void SHT40_Init(void);
uint8_t SHT40_Read_Temperature_Humidity(float *temperature, float *humidity);
uint8_t SHT40_Read_Temperature_Humidity_Ex(uint8_t cmd, float *temperature, float *humidity);
uint32_t SHT40_Read_Serial_Number(void);
uint8_t SHT40_Soft_Reset(void);
uint8_t SHT40_Heater(uint8_t heater_cmd);
void SHT40_Delay(uint32_t ms);

#endif