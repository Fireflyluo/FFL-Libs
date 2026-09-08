/**
 * @file osal_port_stm32.h
 * @brief OSAL（和 sw_timer）需要的 STM32 临界区 port。
 *
 * `ffl.osal` 通过 `osal_port_set_critical_hooks()` 注入临界区入口，
 * `ffl.sw_timer` 通过 `ffl_sw_timer_set_lock_hooks()` 注入同一对入口。
 * 这里用 Cortex-M3 的 PRIMASK（__disable_irq / __enable_irq）实现，
 * 并带嵌套计数，保证 OSAL 回调里再调 OSAL 时不会提前开中断。
 */
#ifndef BSP_OSAL_PORT_STM32_H
#define BSP_OSAL_PORT_STM32_H

#ifdef __cplusplus
extern "C" {
#endif

/** 进入临界区（关中断，支持嵌套）。 */
void bsp_critical_enter(void);

/** 退出临界区（按嵌套深度恢复中断）。 */
void bsp_critical_exit(void);

/**
 * 把本 port 的临界区钩子注册给 `ffl.osal`。
 * tick 钩子不注册：SysTick 常开，stm32f1xx_it.c 每毫秒调用
 * `osal_update_timers()`，无需 OSAL 动态启停硬件定时器。
 */
void bsp_osal_port_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_OSAL_PORT_STM32_H */
