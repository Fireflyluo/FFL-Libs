/**
 * @file board_i2c.h
 * @brief 板级 I2C1 初始化（STM32-LORA：PB8/PB9 重映射）。
 *
 * 属于**应用/board**，不属于 ports/：ports 只消费已 Init 的 HAL 句柄。
 */
#ifndef BSP_BOARD_I2C_H
#define BSP_BOARD_I2C_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化 I2C1 重映射脚 PB8=SCL / PB9=SDA，100kHz。 */
void board_i2c1_init_pb89(void);

/** 返回已就绪的 I2C1 句柄（供 ports/stm32/f1 transport 注入）。 */
I2C_HandleTypeDef *board_i2c1_handle(void);

#ifdef __cplusplus
}
#endif

#endif
