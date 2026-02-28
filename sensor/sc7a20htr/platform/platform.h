/**
 ******************************************************************************
 * @file    sc7a20_platform.h
 * @brief   platform 平台相关代码
 ******************************************************************************
 * @note   根据宏定义切换不同平台实现。
 *
 ******************************************************************************
 */
#ifndef PLATFORM_H
#define PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sc7a20_core.h"

// 平台初始化函数
int accel_init(void);
#if SC7A20_ASYNC_SUPPORT
int accel_init_async(void);
#endif

// 数据读取函数
void read_acceleration_data(void);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_H */