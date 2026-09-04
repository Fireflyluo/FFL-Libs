#ifndef FFL_OSAL_TIMER_PORT_H
#define FFL_OSAL_TIMER_PORT_H

#include "type.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TICK_PERIOD_MS 10U

typedef void (*osal_tick_hook_t)(void);

void osal_port_set_tick_hooks(osal_tick_hook_t init_hook, osal_tick_hook_t start_hook, osal_tick_hook_t stop_hook);
void osal_port_tick_init(void);
void osal_port_tick_start(void);
void osal_port_tick_stop(void);

#define OSAL_TIMER_TICKINIT() osal_port_tick_init()
#define OSAL_TIMER_TICKSTART() osal_port_tick_start()
#define OSAL_TIMER_TICKSTOP() osal_port_tick_stop()

#ifdef __cplusplus
}
#endif

#endif /* FFL_OSAL_TIMER_PORT_H */
