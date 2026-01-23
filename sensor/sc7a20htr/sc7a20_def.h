/**
 ******************************************************************************
 * @file    SC7A20_def.h
 * @brief   SC7A20HTR 基础定义层
 ******************************************************************************
 * @note    本文件内定义驱动的基础数据类型和公共常量
 *
 ******************************************************************************
 */
#ifndef SC7A20_DEF_H
#define SC7A20_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "sc7a20_reg.h"
/* ========================== 常量定义 ========================== */
#define SC7A20_CHIP_ID    0x11 // 设备ID寄存器预期值
#define SC7A20_I2C_ADDR_L 0x18 // SDO接逻辑低
#define SC7A20_I2C_ADDR_H 0x19 // SDO悬空/接逻辑高

/* ========================== 结构体定义 ========================== */

// 设备配置结构
typedef struct {
    uint8_t i2c_addr;          // I2C地址
    sc7a20_accel_fs_t range;   // 量程范围
    sc7a20_accel_odr_t odr;    // 输出数据率
    uint8_t enable_axis[3];    // 三轴使能配置
    bool block_data_update;    // 块数据更新使能
    bool high_resolution_mode; // 高分辨率模式
    bool low_power_mode;       // 低功耗模式
} sc7a20_config_t;

// 加速度数据结构
typedef struct {
    int16_t x;          // 原始X轴数据
    int16_t y;          // 原始Y轴数据
    int16_t z;          // 原始Z轴数据
    float x_g;          // X轴重力加速度值
    float y_g;          // Y轴重力加速度值
    float z_g;          // Z轴重力加速度值
    uint32_t timestamp; // 时间戳（可选）
} sc7a20_accel_data_t;

// FIFO配置结构
typedef struct {
    sc7a20_fifo_mode_t mode; // FIFO模式
    uint8_t watermark;       // 水位线阈值
    bool fifo_enable;        // FIFO使能
} sc7a20_fifo_config_t;

// 中断配置结构
typedef struct {
    uint8_t int_num;   // 中断号(1或2)
    uint8_t threshold; // 中断阈值
    uint8_t duration;  // 中断持续时间
    bool latch_enable; // 锁存使能
    bool active_high;  // 高电平有效
} sc7a20_int_config_t;

// 敲击检测配置
typedef struct {
    bool x_enable;           // X轴敲击检测使能
    bool y_enable;           // Y轴敲击检测使能
    bool z_enable;           // Z轴敲击检测使能
    uint8_t click_threshold; // 敲击阈值
    bool latch_interrupt;    // 锁存中断
} sc7a20_click_config_t;

/* ========================== 枚举类型定义 ========================== */

// 状态码定义
typedef enum {
    SC7A20_OK = 0,          // 成功
    SC7A20_ERROR,           // 一般错误
    SC7A20_TIMEOUT,         // 超时错误
    SC7A20_INVALID_PARAM,   // 无效参数错误
    SC7A20_NOT_INIT,        // 未初始化错误
    SC7A20_COMM_ERROR,      // 通信错误
    SC7A20_DEVICE_NOT_FOUND // 设备未找到
} sc7a20_status_t;

// 电源模式枚举
typedef enum {
    SC7A20_POWER_DOWN = 0, // 电源关断模式
    SC7A20_LOW_POWER,      // 低功耗模式
    SC7A20_NORMAL,         // 正常模式
    SC7A20_HIGH_RESOLUTION // 高分辨率模式
} sc7a20_power_mode_t;

// 数据就绪状态
typedef enum {
    SC7A20_DATA_NOT_READY = 0, // 数据未就绪
    SC7A20_DATA_READY          // 数据就绪
} sc7a20_data_ready_t;

#ifdef __cplusplus
}
#endif

#endif /* SC7A20_DEF_H */