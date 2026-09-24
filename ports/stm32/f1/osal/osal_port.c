/**
 * @file osal_port.c
 * @brief STM32F1 南向：ffl.osal / ffl.sw_timer 临界区（PRIMASK 可嵌套）。
 *
 * 不绑定定时器实例、不含业务；应用在 main 调 ffl_stm32f1_osal_port_init()。
 * sw_timer 可复用同一对 critical_enter/exit。
 */
#include "ffl_port_stm32f1_osal.h"

#include "stm32f1xx_hal.h"
#include "type.h"

static uint32_t s_depth;
static uint32_t s_primask;

void ffl_stm32f1_critical_enter(void) {
  if (s_depth == 0u) {
    s_primask = __get_PRIMASK();
    __disable_irq();
  }
  s_depth++;
}

void ffl_stm32f1_critical_exit(void) {
  if (s_depth > 0u) {
    s_depth--;
    if ((s_depth == 0u) && (s_primask == 0u)) {
      __enable_irq();
    }
  }
}

void ffl_stm32f1_osal_port_init(void) {
  osal_port_set_critical_hooks(ffl_stm32f1_critical_enter,
                               ffl_stm32f1_critical_exit);
}
