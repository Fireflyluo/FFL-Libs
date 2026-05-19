#ifndef ADHOC_PORT_CH32_H
#define ADHOC_PORT_CH32_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t adhoc_port_ch32_now_tc(void);
uint32_t adhoc_port_ch32_now_us(uint32_t tc_us);
uint16_t adhoc_port_ch32_rand_u16(uint32_t *state, uint32_t seed_hint);
void adhoc_port_ch32_critical_enter(void);
void adhoc_port_ch32_critical_exit(void);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_PORT_CH32_H */
