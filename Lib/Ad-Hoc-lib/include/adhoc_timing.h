#ifndef ADHOC_TIMING_H
#define ADHOC_TIMING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADHOC_TMOS_TICK_US 625u
#define ADHOC_TIMING_M_RECOMMENDED_MIN 10u
#define ADHOC_TIMING_M_RECOMMENDED_MAX 1000u

typedef struct
{
    uint32_t t1_us;
    uint32_t t2_us;
    uint32_t t3_us;
    uint32_t t4_us;
} adhoc_timing_cfg_t;

typedef struct
{
    uint32_t t1_us;
    uint32_t t2_us;
    uint32_t t3_us;
    uint32_t t4_us;
    uint32_t t5_us;
    uint32_t t6_us;
    uint16_t n_slots;
    uint16_t m_wait;
} adhoc_timing_plan_t;

int adhoc_timing_build(const adhoc_timing_cfg_t *cfg, adhoc_timing_plan_t *out_plan);
int adhoc_timing_slot_parity_match(uint8_t level, uint16_t slot_no);
int adhoc_timing_is_m_recommended(uint16_t m_wait);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_TIMING_H */
