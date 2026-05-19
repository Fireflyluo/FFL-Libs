#include "adhoc_timing.h"

#include <limits.h>

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
        return 0;
    }
    if (cfg->t3_us <= t5_us)
    {
        return 0;
    }
    if (!adhoc_div_exact_u16(t5_us, cfg->t1_us, &n_slots))
    {
        return 0;
    }
    if (!adhoc_div_exact_u16(t6_us, cfg->t3_us, &m_wait))
    {
        return 0;
    }

    out_plan->t1_us = cfg->t1_us;
    out_plan->t2_us = cfg->t2_us;
    out_plan->t3_us = cfg->t3_us;
    out_plan->t4_us = cfg->t4_us;
    out_plan->t5_us = t5_us;
    out_plan->t6_us = t6_us;
    out_plan->n_slots = n_slots;
    out_plan->m_wait = m_wait;
    return 1;
}

int adhoc_timing_slot_parity_match(uint8_t level, uint16_t slot_no)
{
    return (((uint8_t)slot_no ^ level) & 0x01u) == 0u ? 1 : 0;
}

int adhoc_timing_is_m_recommended(uint16_t m_wait)
{
    return m_wait >= ADHOC_TIMING_M_RECOMMENDED_MIN && m_wait <= ADHOC_TIMING_M_RECOMMENDED_MAX ? 1 : 0;
}
