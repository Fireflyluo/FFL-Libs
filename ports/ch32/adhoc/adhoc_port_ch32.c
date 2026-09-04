#include "adhoc_port_ch32.h"

#include "aros_rf.h"
#include "ch32v20x.h"

#include <stdlib.h>

uint32_t adhoc_port_ch32_now_tc(void)
{
    return (uint32_t)arf_get_tc();
}

uint32_t adhoc_port_ch32_now_us(uint32_t tc_us)
{
    uint32_t tick_us = tc_us;

    if (tick_us == 0u)
    {
        tick_us = (uint32_t)ARF_TC_us;
    }
    return adhoc_port_ch32_now_tc() * tick_us;
}

uint16_t adhoc_port_ch32_rand_u16(uint32_t *state, uint32_t seed_hint)
{
    static uint8_t s_rand_seeded = 0u;
    uint32_t seed = seed_hint;
    uint16_t out;

    if (state != NULL && *state != 0u)
    {
        seed ^= *state;
    }
    if (s_rand_seeded == 0u)
    {
        seed ^= (adhoc_port_ch32_now_tc() << 16) ^ 0xA5A55A5Au;
        if (seed == 0u)
        {
            seed = 1u;
        }
        srand((unsigned int)seed);
        s_rand_seeded = 1u;
    }

    out = (uint16_t)((uint32_t)rand() & 0xFFFFu);
    if (state != NULL)
    {
        *state = ((*state << 16) ^ (uint32_t)out ^ adhoc_port_ch32_now_tc());
    }
    return out;
}

void adhoc_port_ch32_critical_enter(void)
{
    __disable_irq();
}

void adhoc_port_ch32_critical_exit(void)
{
    __enable_irq();
}
