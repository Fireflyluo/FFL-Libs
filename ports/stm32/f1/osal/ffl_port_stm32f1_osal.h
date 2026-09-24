/**
 * @file ffl_port_stm32f1_osal.h
 * @brief `ffl.osal` / `ffl.sw_timer` 在 STM32F1 上的临界区南向适配。
 */
#ifndef FFL_PORT_STM32F1_OSAL_H
#define FFL_PORT_STM32F1_OSAL_H

#ifdef __cplusplus
extern "C" {
#endif

void ffl_stm32f1_critical_enter(void);
void ffl_stm32f1_critical_exit(void);

/**
 * 注册临界区钩子到 ffl.osal。
 * SysTick 侧由应用每 1ms 调 osal_update_timers()，本 port 不管理 tick 硬件。
 */
void ffl_stm32f1_osal_port_init(void);

#ifdef __cplusplus
}
#endif

#endif
