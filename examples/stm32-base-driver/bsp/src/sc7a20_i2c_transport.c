/**
 * @file sc7a20_i2c_transport.c
 * @brief SC7A20 的 STM32 I2C1 轮询 transport（`ffl.driver_port` 契约）。
 *
 * SC7A20 core 只产生两种消息序列（见 sc7a20_core.c）：
 *   - 读寄存器 [WRITE reg][READ data(len)]  -> HAL_I2C_Mem_Read
 *   - 写寄存器 [WRITE reg][WRITE data(len)] -> HAL_I2C_Mem_Write
 * 本实现把这"写寄存器地址 + 数据"映射到 HAL 的寄存器读/写 API，
 * 二者在 F1 上按 START/addrW/reg/(re)START/addrR/data/STOP 时序完成。
 *
 * 只支持同步事务（done == NULL）；异步请求返回 -ENOTSUP。
 * 失败返回负数 errno（-EIO / -ETIMEDOUT / -EINVAL），与 core 的
 * sc7a20_core_map_bus_status() 兼容。
 */
#include "sc7a20_i2c_transport.h"

#include "board.h" /* Error_Handler */

#include <errno.h>

#define I2C_TIMEOUT_MS 50u
#define I2C_DEV_ADDR(addr7) ((uint16_t)((uint16_t)(addr7) << 1))

static I2C_HandleTypeDef s_hi2c1;

static int stm32_i2c_xfer(void *ctx, const ffl_endpoint_t *endpoint,
                          const ffl_xfer_msg_t *msgs, uint8_t count,
                          ffl_xfer_done_fn done, void *user) {
  uint16_t dev_addr;
  HAL_StatusTypeDef hal_status;
  uint8_t is_read;
  uint8_t is_write;

  (void)ctx;
  (void)user;

  /* 本 port 只做同步传输 */
  if (done != NULL) {
    return -ENOTSUP;
  }
  if ((endpoint == NULL) || (endpoint->kind != FFL_ENDPOINT_I2C_7BIT) ||
      (msgs == NULL) || (count == 0u)) {
    return -EINVAL;
  }
  /* core 只发 [写寄存器][写/读数据] 两条消息 */
  if (count != 2u || msgs[0].len != 1u ||
      (msgs[0].flags & FFL_XFER_MSG_WRITE) == 0u) {
    return -EINVAL;
  }

  dev_addr = I2C_DEV_ADDR(endpoint->value.i2c.addr7);
  is_read = (uint8_t)((msgs[1].flags & FFL_XFER_MSG_READ) != 0u ? 1u : 0u);
  is_write = (uint8_t)((msgs[1].flags & FFL_XFER_MSG_WRITE) != 0u ? 1u : 0u);

  if (is_read) {
    hal_status = HAL_I2C_Mem_Read(&s_hi2c1, dev_addr, msgs[0].buf[0],
                                  I2C_MEMADD_SIZE_8BIT, msgs[1].buf,
                                  msgs[1].len, I2C_TIMEOUT_MS);
  } else if (is_write) {
    hal_status = HAL_I2C_Mem_Write(&s_hi2c1, dev_addr, msgs[0].buf[0],
                                   I2C_MEMADD_SIZE_8BIT, (uint8_t *)msgs[1].buf,
                                   msgs[1].len, I2C_TIMEOUT_MS);
  } else {
    return -EINVAL;
  }

  if (hal_status == HAL_OK) {
    return 0;
  }
  if (hal_status == HAL_TIMEOUT) {
    return -ETIMEDOUT;
  }
  return -EIO;
}

static int stm32_i2c_cancel(void *ctx) {
  (void)ctx;
  return 0;
}

static const ffl_transport_ops_t s_stm32_i2c_ops = {
    .xfer = stm32_i2c_xfer,
    .cancel = stm32_i2c_cancel,
};

static ffl_transport_t s_sc7a20_transport = {
    .ops = &s_stm32_i2c_ops,
    .ctx = NULL,
    .endpoint = {.kind = FFL_ENDPOINT_I2C_7BIT, .value.i2c.addr7 = 0x19u},
};

void bsp_sc7a20_i2c_init(void) {
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE(); /* I2C1 使用默认映射 PB6/PB7，仍使能 AFIO 时钟 */
  __HAL_RCC_I2C1_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
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
  if (HAL_I2C_Init(&s_hi2c1) != HAL_OK) {
    Error_Handler();
  }
}

const ffl_transport_t *bsp_sc7a20_transport(void) {
  return &s_sc7a20_transport;
}
