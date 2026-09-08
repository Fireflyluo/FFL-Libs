/**
 * @file sc7a20_i2c_transport.h
 * @brief SC7A20 的 STM32 I2C1 southbound transport。
 *
 * 把 `ffl.driver_port` 的 `ffl_transport_t`（endpoint + msgs + done）
 * 落到 STM32 HAL 的轮询 I2C1（PB6=SCL, PB7=SDA, 100kHz）上。
 *
 * 本 port 只支持同步事务（done == NULL），因为示例在主循环/任务上下文里
 * 做阻塞读。SC7A20 core 构造的消息序列只有两种：
 *   - 读寄存器: [WRITE reg][READ data(len)]  -> HAL_I2C_Mem_Read
 *   - 写寄存器: [WRITE reg][WRITE data(len)] -> HAL_I2C_Mem_Write
 * 二者在 F1 HAL 内部都会产生 START + addr + reg(+restart) + data + STOP，
 * 正是 SC7A20 需要的 I2C 时序，无需额外延时。
 */
#ifndef BSP_SC7A20_I2C_TRANSPORT_H
#define BSP_SC7A20_I2C_TRANSPORT_H

#include "ffl/driver_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化并启动 I2C1（时钟 + PB6/PB7 复用开漏 + 100kHz），在 board_init
 * 中调用。 */
void bsp_sc7a20_i2c_init(void);

/** 返回 SC7A20 使用的 transport 实例。 */
const ffl_transport_t *bsp_sc7a20_transport(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SC7A20_I2C_TRANSPORT_H */
