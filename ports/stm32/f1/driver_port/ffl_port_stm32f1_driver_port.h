/**
 * @file ffl_port_stm32f1_driver_port.h
 * @brief STM32F1 南向：时间能力 + 通用 I2C `ffl_transport_t`。
 *
 * 适配层**不**初始化 I2C 外设、**不**选引脚。应用/board 负责：
 *   时钟、GPIO 复用、HAL_I2C_Init；
 * 然后把已就绪的 `I2C_HandleTypeDef*` 注入本 port。
 *
 * 同一芯片可支持 I2C1/I2C2：各自准备 handle 与 transport 实例。
 */
#ifndef FFL_PORT_STM32F1_DRIVER_PORT_H
#define FFL_PORT_STM32F1_DRIVER_PORT_H

#include "ffl/driver_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/** DWT 自由运行计数（芯片级，调用一次即可）。 */
void ffl_stm32f1_time_init(void);

/** `ffl_time_ops_t`：delay_ms=HAL_Delay，delay_us/now_us=DWT。 */
const ffl_time_ops_t *ffl_stm32f1_time_ops(void);

/**
 * I2C 同步 transport 的用户上下文（必须在 transport 使用期间保持有效）。
 */
typedef struct {
  void *hi2c;        /**< I2C_HandleTypeDef* */
  uint32_t timeout_ms; /**< HAL 超时，0 则默认 50ms */
} ffl_stm32f1_i2c_ctx_t;

/** `ffl_transport_ops_t` 实现（同步 xfer，done==NULL）。 */
const ffl_transport_ops_t *ffl_stm32f1_i2c_ops(void);

/**
 * 把 caller 拥有的 `ffl_transport_t` 填成可用的 I2C transport。
 *
 * @param out   调用方存储，须在绑定设备的整个生命周期内有效
 * @param ctx   调用方存储，须在绑定设备的整个生命周期内有效，且 hi2c 已 HAL_I2C_Init
 * @param addr7 默认 7-bit 地址（驱动可用 set_i2c_addr 再改）
 * @return 0；参数错误返回负值
 */
int ffl_stm32f1_i2c_transport_setup(ffl_transport_t *out,
                                    ffl_stm32f1_i2c_ctx_t *ctx,
                                    uint8_t addr7);

#ifdef __cplusplus
}
#endif

#endif
