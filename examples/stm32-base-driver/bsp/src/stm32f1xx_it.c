/**
 * @file stm32f1xx_it.c
 * @brief 异常/中断向量；SysTick 是组件时间的唯一来源。
 *
 * 设计要点：
 *   - 组件不拥有硬件定时器：SysTick 由本文件统一喂给 HAL / OSAL / sw_timer。
 *   - ISR 里只做「推进」；业务与组件回调在主循环 process 中执行。
 *
 * 时基用途：
 *   HAL_IncTick()           → HAL_Delay / HAL_GetTick
 *   osal_update_timers()    → ffl.osal 的 reload 定时器到期置事件
 *   ffl_sw_timer_tick_isr() → ffl.sw_timer 时间轮槽位前进
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

/** 1ms 系统节拍：三处共用，勿在此调用重业务或阻塞 API。 */
void SysTick_Handler(void) {
  HAL_IncTick();
  osal_update_timers();
  ffl_sw_timer_tick_isr();
}
