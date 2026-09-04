/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_timing.h
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            时序参数模块
**                          计算 T5/T6/n/m 等派生时序参数, 提供时隙奇偶匹配检查
**--------------------------------------------------------------------------------------------------------
*/

#ifndef ADHOC_TIMING_H
#define ADHOC_TIMING_H

#include "adhoc_config.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief TMOS 基础时基(μs) */
#define ADHOC_TMOS_TICK_US ADHOC_CONFIG_TMOS_TICK_US
/** @brief m 推荐最小值 */
#define ADHOC_TIMING_M_RECOMMENDED_MIN ADHOC_CONFIG_TIMING_M_RECOMMENDED_MIN
/** @brief m 推荐最大值 */
#define ADHOC_TIMING_M_RECOMMENDED_MAX ADHOC_CONFIG_TIMING_M_RECOMMENDED_MAX

/**
 * @brief 时序配置(用户输入的基础时序)
 */
typedef struct
{
    uint32_t t1_us; /**< 时隙宽度(μs) */
    uint32_t t2_us; /**< 接收窗口(μs) */
    uint32_t t3_us; /**< 探测侦听(μs) */
    uint32_t t4_us; /**< 探测休眠(μs) */
} adhoc_timing_cfg_t;

/**
 * @brief 时序方案(派生的完整时序参数)
 */
typedef struct
{
    uint32_t t1_us;   /**< 时隙宽度 */
    uint32_t t2_us;   /**< 接收窗口 */
    uint32_t t3_us;   /**< 探测侦听 */
    uint32_t t4_us;   /**< 探测休眠 */
    uint32_t t5_us;   /**< 应答周期(T1+T2) */
    uint32_t t6_us;   /**< 探测周期(T3+T4) */
    uint16_t n_slots; /**< 每应答周期时隙数(T5/T1) */
    uint16_t m_wait;  /**< 探测等待倍数(T6/T3) */
} adhoc_timing_plan_t;

/**
 * @brief 从基础时序构建完整时序方案
 * @param cfg      基础时序参数
 * @param out_plan 输出时序方案
 * @return 1成功 0失败(T1须为625的整数倍, T3>T5等约束)
 */
int adhoc_timing_build(const adhoc_timing_cfg_t *cfg, adhoc_timing_plan_t *out_plan);

/**
 * @brief 检查时隙号与级别的奇偶匹配
 * @param level   节点级别
 * @param slot_no 时隙号
 * @return 1匹配 0不匹配
 */
int adhoc_timing_slot_parity_match(uint8_t level, uint16_t slot_no);

/**
 * @brief 检查 m_wait 是否在推荐范围内
 * @param m_wait 探测等待倍数
 * @return 1在范围内 0超出
 */
int adhoc_timing_is_m_recommended(uint16_t m_wait);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_TIMING_H */
