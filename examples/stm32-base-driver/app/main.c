/**
 * @file main.c
 * @brief 合并示例：OSAL 多任务展示 ffl 组件与板级外设。
 *
 * 板型（STM32-LORA 原板）：
 *   Flash = 软 SPI（PB3/4/5+PA15），不碰 SPI1 remap
 *   LCD   = 硬 SPI1 + DMA TX（PA5/7/4）
 *   USB   = CDC 收图写入 W25Q
 *
 * 任务：
 *   heartbeat / sc7a20 / flash / lcd / features_demo
 */
#include "board.h"
#include "ffl/sw_timer.h"
#include "osal.h"
#include "osal_event.h"
#include "osal_tasks.h"
#include "ffl_port_stm32f1_osal.h"
#include "ulog.h"
#include "usb_img.h"

static int ulog_uart_tx_try(void *ctx, const uint8_t *data, uint16_t len) {
  (void)ctx;
  return board_uart_write_try((const char *)data, (int)len);
}

static void app_ulog_init(void) {
  ulog_init_t cfg;
  cfg.tx_try = ulog_uart_tx_try;
  cfg.tx_direct = NULL;
  cfg.tx_ctx = NULL;
  cfg.poll_budget = 64u;
  (void)ulog_init(&cfg);
}

int main(void) {
  board_init();
  app_ulog_init();

  ffl_stm32f1_osal_port_init();
  ffl_sw_timer_set_lock_hooks(ffl_stm32f1_critical_enter,
                              ffl_stm32f1_critical_exit);
  ffl_sw_timer_wheel_init(1u);

  features_demo_run();
  usb_img_init();
  ULOGI("ulog + USART1 DMA TX + SPI1 DMA TX ready");

  (void)osal_init_system();
  heartbeat_task_register();
  sc7a20_task_register();
  flash_task_register();
  lcd_task_register();
  alarm_task_register();
  osal_Task_init();

  for (;;) {
    osal_process_once();
    ffl_sw_timer_process();
    ulog_poll();
    board_uart_kick_tx();
    board_idle();
  }
}
