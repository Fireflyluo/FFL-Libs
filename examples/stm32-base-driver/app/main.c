/**
 * @file main.c
 * @brief STM32F103C8T6 最小系统板入口。
 *
 * 启动顺序：
 *   1. board_init(): HAL、72MHz、LED、DWT、I2C1；
 *   2. bsp_osal_port_init(): 把 STM32 临界区注册给 ffl.osal；
 *   3. ffl_sw_timer_*: 初始化时间轮并注册同一对临界区钩子；
 *   4. ffl.osal 初始化、注册应用任务并运行其 init；
 *   5. 主循环：osal_process_once() + ffl_sw_timer_process() + WFI。
 *
 * SysTick(1ms) 中断负责 osal_update_timers() 与 ffl_sw_timer_tick_isr()，
 * 见 bsp/src/stm32f1xx_it.c。
 */
#include "board.h"
#include "ffl/sw_timer.h"
#include "osal.h"
#include "osal_event.h"

#include "app_task.h"
#include "osal_port_stm32.h"

int main(void) {
  board_init();

  /* ffl.osal 临界区钩子 */
  bsp_osal_port_init();

  /* ffl.sw_timer：时间轮 1ms 一格，锁钩子复用 STM32 临界区 */
  ffl_sw_timer_set_lock_hooks(bsp_critical_enter, bsp_critical_exit);
  ffl_sw_timer_wheel_init(1u);

  /* ffl.osal 系统初始化 + 任务注册 */
  (void)osal_init_system();
  app_task_register();
  osal_Task_init();

  for (;;) {
    osal_process_once();
    ffl_sw_timer_process(); /* 执行到期 sw_timer 回调（任务上下文） */
    board_idle();
  }
}
