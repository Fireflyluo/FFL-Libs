/**
 ******************************************************************************
 * @file    SC7A20_hal.h
 * @brief   SC7A20HTR 硬件抽象层接口定义
 ******************************************************************************
 * @note    本文件内定义定义硬件操作的标准接口
 *
 ******************************************************************************
 */
#ifndef SC7A20_HAL_H
#define SC7A20_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sc7a20_def.h"

/* ========================== 函数指针类型定义 ========================== */

/**
 * @brief 写寄存器函数指针类型
 * @param reg 寄存器地址
 * @param data 要写入的数据缓冲区
 * @param len 数据长度
 * @return 操作状态
 */
typedef sc7a20_status_t (*sc7a20_hal_write_fn)(uint8_t reg, const uint8_t *data, uint16_t len);

/**
 * @brief 读寄存器函数指针类型
 * @param reg 寄存器地址
 * @param data 读取数据的缓冲区
 * @param len 数据长度
 * @return 操作状态
 */
typedef sc7a20_status_t (*sc7a20_hal_read_fn)(uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief 延时函数指针类型
 * @param ms 延时毫秒数
 */
typedef void (*sc7a20_hal_delay_fn)(uint32_t ms);

#if SC7A20_ASYNC_SUPPORT
/**
 * @brief 异步写寄存器函数指针类型
 * @param reg 寄存器地址
 * @param data 要写入的数据缓冲区
 * @param len 数据长度
 * @param callback 完成回调函数
 * @return 操作状态
 */
typedef sc7a20_status_t (*sc7a20_hal_write_async_fn)(uint8_t reg, const uint8_t *data, uint16_t len,
                                                   void (*callback)(void *user_data, sc7a20_status_t status));

/**
 * @brief 异步读寄存器函数指针类型
 * @param reg 寄存器地址
 * @param data 读取数据的缓冲区
 * @param len 数据长度
 * @param callback 完成回调函数
 * @return 操作状态
 */
typedef sc7a20_status_t (*sc7a20_hal_read_async_fn)(uint8_t reg, uint8_t *data, uint16_t len,
                                                  void (*callback)(void *user_data, sc7a20_status_t status));

/**
 * @brief 检查硬件是否忙的函数指针类型
 * @return true表示忙，false表示空闲
 */
typedef bool (*sc7a20_hal_check_busy_fn)(void);
#endif

/* ========================== 硬件操作接口结构体 ========================== */

/**
 * @brief 硬件操作接口结构体
 */
typedef struct {
    sc7a20_hal_write_fn write;    // 写寄存器函数
    sc7a20_hal_read_fn read;      // 读寄存器函数
    sc7a20_hal_delay_fn delay_ms; // 延时函数
    
#if SC7A20_ASYNC_SUPPORT
    // 异步操作接口（新增）
    sc7a20_hal_write_async_fn write_async; // 异步写寄存器函数
    sc7a20_hal_read_async_fn read_async;   // 异步读寄存器函数
    sc7a20_hal_check_busy_fn is_busy;      // 检查硬件是否忙
#endif
    
    void *user_data;              // 用户可携带任意数据（如I2C句柄）
} sc7a20_ops_t;

/* ========================== 硬件相关常量 ========================== */

// 通信超时时间（毫秒）
#define SC7A20_COMM_TIMEOUT_MS 100

// 最大重试次数
#define SC7A20_MAX_RETRY_COUNT 3

#ifdef __cplusplus
}
#endif

#endif /* SC7A20_HAL_H */