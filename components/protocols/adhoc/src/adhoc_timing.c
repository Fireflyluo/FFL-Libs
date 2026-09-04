/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_timing.c
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            时序参数实现
**                          时序方案构建(T5/T6/n/m计算)、时隙奇偶匹配检查
**--------------------------------------------------------------------------------------------------------
*/

#include "adhoc_timing.h"

#include <limits.h>

/**
 * @brief 精确整数除法(要求整除, 且结果在 uint16_t 范围内)
 * @param numerator   分子
 * @param denominator 分母
 * @param out_value   输出商
 * @return 1成功 0失败(不能整除或溢出)
 */
static int adhoc_div_exact_u16(uint32_t numerator, uint32_t denominator, uint16_t *out_value)
{
    uint32_t value;

    if (denominator == 0u || out_value == 0)
    {
        return 0;
    }
    if ((numerator % denominator) != 0u)
    {
        return 0;
    }

    value = numerator / denominator;
    if (value == 0u || value > UINT16_MAX)
    {
        return 0;
    }

    *out_value = (uint16_t)value;
    return 1;
}

/**
 * @brief 从基础时序构建完整时序方案
 *
 * 约束: T1 必须是 625μs 的整数倍, T3 > T5, 且各派生参数均为整数
 */
int adhoc_timing_build(const adhoc_timing_cfg_t *cfg, adhoc_timing_plan_t *out_plan)
{
    uint32_t t5_us;
    uint32_t t6_us;
    uint16_t n_slots;
    uint16_t m_wait;

    if (cfg == 0 || out_plan == 0)
    {
        return 0;
    }
    if (cfg->t1_us == 0u || cfg->t2_us == 0u || cfg->t3_us == 0u || cfg->t4_us == 0u)
    {
        return 0;
    }
    if ((cfg->t1_us % ADHOC_TMOS_TICK_US) != 0u)
    {
        return 0;
    }

    t5_us = cfg->t1_us + cfg->t2_us;
    t6_us = cfg->t3_us + cfg->t4_us;
    if (t5_us < cfg->t1_us || t6_us < cfg->t3_us)
    {
        return 0; /* 溢出检查 */
    }
    if (cfg->t3_us <= t5_us)
    {
        return 0; /* T3 必须大于 T5 */
    }
    if (!adhoc_div_exact_u16(t5_us, cfg->t1_us, &n_slots))
    {
        return 0;
    }
    if (!adhoc_div_exact_u16(t6_us, cfg->t3_us, &m_wait))
    {
        return 0;
    }

    out_plan->t1_us   = cfg->t1_us;
    out_plan->t2_us   = cfg->t2_us;
    out_plan->t3_us   = cfg->t3_us;
    out_plan->t4_us   = cfg->t4_us;
    out_plan->t5_us   = t5_us;
    out_plan->t6_us   = t6_us;
    out_plan->n_slots = n_slots;
    out_plan->m_wait  = m_wait;
    return 1;
}

/**
 * @brief 检查时隙号的奇偶匹配: (slot_no XOR level) 的最低位必须为0
 * @return 1匹配 0不匹配
 */
int adhoc_timing_slot_parity_match(uint8_t level, uint16_t slot_no)
{
    return (((uint8_t)slot_no ^ level) & 0x01u) == 0u ? 1 : 0;
}

/**
 * @brief 检查 m_wait 是否在推荐范围 [10, 1000]
 */
int adhoc_timing_is_m_recommended(uint16_t m_wait)
{
    return m_wait >= ADHOC_TIMING_M_RECOMMENDED_MIN && m_wait <= ADHOC_TIMING_M_RECOMMENDED_MAX ? 1 : 0;
}
