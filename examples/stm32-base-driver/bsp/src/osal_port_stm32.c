/**
 * @file osal_port_stm32.c
 * @brief `ffl.osal` / `ffl.sw_timer` 的 STM32 临界区实现。
 *
 * 用 PRIMASK 实现可嵌套临界区：首次进入关中断并记录原 PRIMASK，
 * 最外层退出时按记录恢复。
 */
#include "osal_port_stm32.h"

#include "stm32f1xx_hal.h" /* __disable_irq / __enable_irq / __get_PRIMASK */
#include "type.h"          /* osal_port_set_critical_hooks */

static uint32_t s_critical_depth;
static uint32_t s_primask_saved;

void bsp_critical_enter(void) {
  if (s_critical_depth == 0u) {
    s_primask_saved = __get_PRIMASK();
    __disable_irq();
  }
  s_critical_depth++;
}

void bsp_critical_exit(void) {
  if (s_critical_depth > 0u) {
    s_critical_depth--;
    if ((s_critical_depth == 0u) && (s_primask_saved == 0u)) {
      __enable_irq();
    }
  }
}

void bsp_osal_port_init(void) {
  osal_port_set_critical_hooks(bsp_critical_enter, bsp_critical_exit);
  /* tick 钩子不注册：SysTick 常开，stm32f1xx_it.c 每毫秒调用
   * osal_update_timers()，不需要 OSAL 动态启停硬件定时器。 */
}
