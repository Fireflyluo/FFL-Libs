/**
 * @file stm32_time_ops.h
 * @brief `ffl.driver_port` 时间能力在 STM32 上的实现。
 *
 * 提供 `ffl_time_ops_t`：
 *   - delay_ms: 复用 HAL_Delay（SysTick 1ms）
 *   - delay_us / now_us: 基于 DWT->CYCCNT 自由运行计数（board_init 里
 *     通过 bsp_time_init() 使能），时钟 72MHz，单位换算自 SystemCoreClock。
 */
#ifndef BSP_STM32_TIME_OPS_H
#define BSP_STM32_TIME_OPS_H

#include "ffl/driver_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 使能 DWT 周期计数（在 board_init 中调用）。 */
void bsp_time_init(void);

/** 返回供 `ffl_sc7a20_bind()` 使用的 time ops。 */
const ffl_time_ops_t *bsp_time_ops(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_STM32_TIME_OPS_H */
