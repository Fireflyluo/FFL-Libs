/**
 * @file stm32f1xx_it.c
 * @brief STM32F103C8T6 中断入口。
 *
 * SysTick_Handler 每 1ms 被调用，同一时基喂给三处：
 *   1. HAL_IncTick()      -> HAL_Delay / HAL_GetTick
 *   2. osal_update_timers()-> ffl.osal 的任务定时器与系统时钟
 *   3. ffl_sw_timer_tick_isr()-> ffl.sw_timer 时间轮推进
 * 到期的 sw_timer 回调由主循环的 ffl_sw_timer_process() 在任务上下文执行。
 */
#include "stm32f1xx_hal.h"

#include "ffl/sw_timer.h"
#include "osal_timer.h"

void NMI_Handler(void) {
  while (1) {
  }
}

void HardFault_Handler(void) {
  while (1) {
  }
}

void MemManage_Handler(void) {
  while (1) {
  }
}

void BusFault_Handler(void) {
  while (1) {
  }
}

void UsageFault_Handler(void) {
  while (1) {
  }
}

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

void PendSV_Handler(void) {}

void SysTick_Handler(void) {
  HAL_IncTick();
  osal_update_timers();
  ffl_sw_timer_tick_isr();
}
