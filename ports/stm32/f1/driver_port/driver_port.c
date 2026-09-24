/**
 * @file driver_port.c
 * @brief STM32F1 南向：DWT 时间 + 通用 I2C 同步 transport。
 *
 * 关键约定（适配层通用性）：
 *   - 不调用 HAL_I2C_Init，不配置 GPIO；
 *   - 应用填 ffl_stm32f1_i2c_ctx_t.hi2c（已 Init 的 I2C_HandleTypeDef*）；
 *   - ffl_stm32f1_i2c_transport_setup() 只绑定 ops/ctx/默认地址；
 *   - xfer() 把驱动的 [WRITE reg][READ|WRITE data] 映射到 HAL_I2C_Mem_*。
 *
 * 因此同一 ops 可服务 I2C1、I2C2 等多个实例（各自一份 ctx + transport）。
 */
#include "ffl_port_stm32f1_driver_port.h"

#include "stm32f1xx_hal.h"

#include <errno.h>

void ffl_stm32f1_time_init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t now_us(void *ctx) {
  (void)ctx;
  if (SystemCoreClock == 0u) {
    return 0u;
  }
  return (uint32_t)(((uint64_t)DWT->CYCCNT * 1000000u) / SystemCoreClock);
}

static void delay_us(void *ctx, uint32_t us) {
  uint32_t start;
  (void)ctx;
  start = now_us(NULL);
  while ((int32_t)(now_us(NULL) - start) < (int32_t)us) {
  }
}

static void delay_ms(void *ctx, uint32_t ms) {
  (void)ctx;
  HAL_Delay(ms);
}

static const ffl_time_ops_t g_time_ops = {
    .delay_ms = delay_ms,
    .delay_us = delay_us,
    .now_us = now_us,
};

const ffl_time_ops_t *ffl_stm32f1_time_ops(void) { return &g_time_ops; }

/**
 * 同步 I2C：驱动消息形态
 *   [WRITE reg(1B)][READ|WRITE data]
 * 映射为 HAL_I2C_Mem_Read / Mem_Write。
 * ctx = ffl_stm32f1_i2c_ctx_t*（应用注入 I2C_HandleTypeDef*）。
 */
static int i2c_xfer(void *ctx, const ffl_endpoint_t *endpoint,
                    const ffl_xfer_msg_t *msgs, uint8_t count,
                    ffl_xfer_done_fn done, void *user) {
  const ffl_stm32f1_i2c_ctx_t *ictx = (const ffl_stm32f1_i2c_ctx_t *)ctx;
  I2C_HandleTypeDef *hi2c;
  uint16_t dev_addr;
  uint32_t timeout;
  HAL_StatusTypeDef st;

  (void)user;

  if (ictx == NULL || ictx->hi2c == NULL) {
    return -EINVAL;
  }
  if (done != NULL) {
    return -ENOTSUP;
  }
  if (endpoint == NULL || endpoint->kind != FFL_ENDPOINT_I2C_7BIT ||
      msgs == NULL || count == 0u) {
    return -EINVAL;
  }
  if (count != 2u || msgs[0].len != 1u ||
      (msgs[0].flags & FFL_XFER_MSG_WRITE) == 0u) {
    return -EINVAL;
  }

  hi2c = (I2C_HandleTypeDef *)ictx->hi2c;
  timeout = (ictx->timeout_ms != 0u) ? ictx->timeout_ms : 50u;
  dev_addr = (uint16_t)((uint16_t)endpoint->value.i2c.addr7 << 1);

  if ((msgs[1].flags & FFL_XFER_MSG_READ) != 0u) {
    st = HAL_I2C_Mem_Read(hi2c, dev_addr, msgs[0].buf[0],
                          I2C_MEMADD_SIZE_8BIT, msgs[1].buf, msgs[1].len,
                          timeout);
  } else if ((msgs[1].flags & FFL_XFER_MSG_WRITE) != 0u) {
    st = HAL_I2C_Mem_Write(hi2c, dev_addr, msgs[0].buf[0],
                           I2C_MEMADD_SIZE_8BIT, (uint8_t *)msgs[1].buf,
                           msgs[1].len, timeout);
  } else {
    return -EINVAL;
  }

  if (st == HAL_OK) {
    return 0;
  }
  return (st == HAL_TIMEOUT) ? -ETIMEDOUT : -EIO;
}

static int i2c_cancel(void *ctx) {
  (void)ctx;
  return 0;
}

static const ffl_transport_ops_t g_i2c_ops = {
    .xfer = i2c_xfer,
    .cancel = i2c_cancel,
};

const ffl_transport_ops_t *ffl_stm32f1_i2c_ops(void) { return &g_i2c_ops; }

int ffl_stm32f1_i2c_transport_setup(ffl_transport_t *out,
                                    ffl_stm32f1_i2c_ctx_t *ctx,
                                    uint8_t addr7) {
  if (out == NULL || ctx == NULL || ctx->hi2c == NULL) {
    return -1;
  }
  if (addr7 == 0u) {
    return -1;
  }
  out->ops = &g_i2c_ops;
  out->ctx = ctx;
  out->endpoint.kind = FFL_ENDPOINT_I2C_7BIT;
  out->endpoint.value.i2c.addr7 = addr7;
  return 0;
}
