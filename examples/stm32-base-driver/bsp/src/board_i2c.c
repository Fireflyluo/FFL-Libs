/**
 * @file board_i2c.c
 * @brief 板级 I2C1：时钟、PB8/PB9 重映射、HAL_I2C_Init。
 *
 * 与 ports 分工：本文件选脚并 Init；ports/stm32/f1 只拿 handle 做 xfer。
 * 应用通过 board_i2c1_handle() 注入到 ffl_stm32f1_i2c_ctx_t.hi2c。
 */
#include "board_i2c.h"

static I2C_HandleTypeDef s_hi2c1;

void board_i2c1_init_pb89(void) {
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  __HAL_AFIO_REMAP_I2C1_ENABLE();
  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  s_hi2c1.Instance = I2C1;
  s_hi2c1.Init.ClockSpeed = 100000u;
  s_hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  s_hi2c1.Init.OwnAddress1 = 0u;
  s_hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  s_hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  s_hi2c1.Init.OwnAddress2 = 0u;
  s_hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  s_hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  (void)HAL_I2C_Init(&s_hi2c1);
}

I2C_HandleTypeDef *board_i2c1_handle(void) { return &s_hi2c1; }
