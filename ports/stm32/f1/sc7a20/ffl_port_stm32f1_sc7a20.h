/**
 * @file ffl_port_stm32f1_sc7a20.h
 * @brief 用应用注入的 I2C transport 绑定 ffl.sc7a20（薄封装，无外设初始化）。
 */
#ifndef FFL_PORT_STM32F1_SC7A20_H
#define FFL_PORT_STM32F1_SC7A20_H

#include "ffl/sc7a20.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * bind + 设 7-bit 地址。transport 与 time_ops 由调用方准备
 * （time_ops 可用 ffl_stm32f1_time_ops()）。
 *
 * @return 0 成功；否则为 bind/set_i2c_addr 的负错误码
 */
int ffl_stm32f1_sc7a20_bind(ffl_sc7a20_device_t *device,
                            const ffl_transport_t *transport,
                            uint8_t addr7);

#ifdef __cplusplus
}
#endif

#endif
