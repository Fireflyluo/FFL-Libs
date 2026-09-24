/**
 * @file usb_port_stm32f1.c
 * @brief STM32-LORA 板：CherryUSB fsdev 的时钟 / GPIO / NVIC 适配。
 *
 * 由上游 port/fsdev/usb_glue_st.c 的 usb_dc_low_level_init() 调用
 * HAL_PCD_MspInit()。此处提供 F103 板级实现，不修改 upstream。
 *
 * USB 时钟：SYSCLK 72MHz -> PLL/1.5 = 48MHz。
 * 引脚：PA11=USB_DM, PA12=USB_DP（片上 USB 外设，默认浮空输入）。
 * 中断：USB_LP_CAN1_RX0（F103 medium-density）。
 */
#include "stm32f1xx_hal.h"

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd) {
  (void)hpcd;

  __HAL_RCC_USB_CLK_ENABLE();
  /* F103：USBPRE=0 → USBCLK = PLL/1.5 = 48MHz（72MHz PLL）。 */
  MODIFY_REG(RCC->CFGR, RCC_CFGR_USBPRE, 0U);
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd) {
  (void)hpcd;
  HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  __HAL_RCC_USB_CLK_DISABLE();
}
