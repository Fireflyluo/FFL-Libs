/**
 ******************************************************************************
 * @file    sc7a20_async.h
 * @brief   SC7A20HTR 异步操作辅助接口
 ******************************************************************************
 * @note    本文件定义异步操作的内部辅助函数和状态机处理接口
 *
 ******************************************************************************
 */
#ifndef SC7A20_ASYNC_H
#define SC7A20_ASYNC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "sc7a20_core.h"

#if SC7A20_ASYNC_SUPPORT

/* ========================== 内部辅助函数 ========================== */

/**
 * @brief 异步操作状态机处理函数
 * @param handle 设备句柄
 */
void sc7a20_process_async_operation(sc7a20_handle_t handle);

/**
 * @brief 检查异步操作是否超时
 * @param handle 设备句柄
 * @return true表示超时，false表示未超时
 */
bool sc7a20_check_async_timeout(sc7a20_handle_t handle);

/**
 * @brief 转换原始数据到加速度数据结构
 * @param handle 设备句柄
 * @param raw_data 原始数据缓冲区
 * @param accel_data 加速度数据输出结构
 */
void sc7a20_convert_raw_to_accel_data(sc7a20_handle_t handle, 
                                    const uint8_t *raw_data, 
                                    sc7a20_accel_data_t *accel_data);

/**
 * @brief 内部读完成回调函数
 * @param user_data 用户数据（设备句柄）
 * @param status 操作状态
 */
void sc7a20_internal_read_callback(void *user_data, sc7a20_status_t status);

/**
 * @brief 内部写完成回调函数
 * @param user_data 用户数据（设备句柄）
 * @param status 操作状态
 */
void sc7a20_internal_write_callback(void *user_data, sc7a20_status_t status);

/* ========================== 配置常量 ========================== */

// 默认异步操作超时时间（毫秒）
#define SC7A20_ASYNC_DEFAULT_TIMEOUT_MS 100

// 内部缓冲区大小
#define SC7A20_INTERNAL_BUFFER_SIZE 6

#endif // SC7A20_ASYNC_SUPPORT

#ifdef __cplusplus
}
#endif

#endif /* SC7A20_ASYNC_H */