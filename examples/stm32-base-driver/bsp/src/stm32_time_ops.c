/**
 * @file stm32_time_ops.c
 * @brief `ffl.driver_port` 时间能力（delay_ms / delay_us / now_us）。
 *
 * delay_ms 复用 HAL_Delay（SysTick 1ms）。
 * now_us 读取 DWT->CYCCNT；时钟经 SystemCoreClock 换算为微秒，自
 * bsp_time_init() 使能后自由运行，供短延时与超时判断使用。
 */
#include "stm32_time_ops.h"

#include "stm32f1xx_hal.h"

void bsp_time_init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t stm32_now_us(void *ctx) {
  (void)ctx;
  if (SystemCoreClock == 0u) {
    return 0u;
  }
  return (uint32_t)(((uint64_t)DWT->CYCCNT * 1000000u) / SystemCoreClock);
}

static void stm32_delay_us(void *ctx, uint32_t us) {
  uint32_t start;

  (void)ctx;
  start = stm32_now_us(NULL);
  while ((int32_t)(stm32_now_us(NULL) - start) < (int32_t)us) {
  }
}

static void stm32_delay_ms(void *ctx, uint32_t ms) {
  (void)ctx;
  HAL_Delay(ms);
}

static const ffl_time_ops_t g_stm32_time_ops = {
    .delay_ms = stm32_delay_ms,
    .delay_us = stm32_delay_us,
    .now_us = stm32_now_us,
};

const ffl_time_ops_t *bsp_time_ops(void) { return &g_stm32_time_ops; }
