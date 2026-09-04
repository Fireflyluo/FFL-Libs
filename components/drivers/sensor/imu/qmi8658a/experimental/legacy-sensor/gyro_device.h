/**
 ******************************************************************************
 * @file    gyro_device.h
 * @brief   陀螺仪设备统一接口定义
 ******************************************************************************
 */

#ifndef GYRO_DEVICE_H
#define GYRO_DEVICE_H

#include <stdint.h>

/* 陀螺仪量程枚举 */
typedef enum {
    GYRO_RANGE_250DPS = 0,    // ±250 dps
    GYRO_RANGE_500DPS = 1,    // ±500 dps
    GYRO_RANGE_1000DPS = 2,   // ±1000 dps
    GYRO_RANGE_2000DPS = 3    // ±2000 dps
} gyro_range_t;

/* 陀螺仪输出数据率枚举 */
typedef enum {
    GYRO_ODR_100HZ = 0,       // 100 Hz
    GYRO_ODR_200HZ = 1,       // 200 Hz
    GYRO_ODR_400HZ = 2,       // 400 Hz
    GYRO_ODR_800HZ = 3        // 800 Hz
} gyro_odr_t;

/* 传感器数据结构体 */
typedef struct {
    float accel[3];           // 加速度数据 [X, Y, Z] (m/s²)
    float gyro[3];            // 陀螺仪数据 [X, Y, Z] (rad/s)
    float temp;               // 温度 (°C)
} sensor_data_t;

/* 陀螺仪设备接口结构体 */
typedef struct {
    int32_t (*init)(void *hardware_handle);                // 初始化函数
    int32_t (*read_data)(sensor_data_t *data);             // 读取传感器数据
    int32_t (*set_range)(gyro_range_t range);              // 设置量程
    int32_t (*set_odr)(gyro_odr_t odr);                   // 设置输出数据率
    int32_t (*sleep)(void);                                // 进入休眠模式
    int32_t (*wakeup)(void);                               // 唤醒设备
} gyro_device_t;

#endif /* GYRO_DEVICE_H */