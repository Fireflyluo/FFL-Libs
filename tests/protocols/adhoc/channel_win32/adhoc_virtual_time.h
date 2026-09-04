/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_virtual_time.h
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            虚拟时间模块
**                          基于 Windows QueryPerformanceCounter 的高精度单调微秒时钟,
**                          全仿真共享同一实例以确保时序一致性。实现内联在头文件中。
**--------------------------------------------------------------------------------------------------------
*/

#ifndef ADHOC_VIRTUAL_TIME_H
#define ADHOC_VIRTUAL_TIME_H

#include <stdint.h>
#include <windows.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 虚拟时间实例(含 QPC 频率)
 */
typedef struct
{
    LARGE_INTEGER freq; /**< QPC 频率(计数/秒) */
} adhoc_virtual_time_t;

/**
 * @brief 初始化虚拟时间(获取 QPC 频率)
 * @param vt 虚拟时间实例
 */
static void adhoc_virtual_time_init(adhoc_virtual_time_t *vt)
{
    QueryPerformanceFrequency(&vt->freq);
}

/**
 * @brief 获取当前微秒时间戳
 * @param vt 虚拟时间实例
 * @return 单调递增的微秒时间戳
 */
static uint32_t adhoc_virtual_time_now_us(adhoc_virtual_time_t *vt)
{
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return (uint32_t)((now.QuadPart * 1000000ULL) / (unsigned long long)vt->freq.QuadPart);
}

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_VIRTUAL_TIME_H */
