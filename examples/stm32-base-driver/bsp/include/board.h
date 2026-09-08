/**
 * @file board.h
 * @brief STM32F103C8T6 最小系统板板级接口。
 *
 * 该文件是示例自有的板级层，不属于仓库组件。它把 STM32 HAL 初始化的
 * 结果（时钟、LED、SysTick、DWT、I2C1）包装成 `app/` 需要的少量函数，
 * 以及 `ffl.driver_port` 期望的 southbound 能力（见同目录下其它头）。
 *
 * 接线约定（标准 STM32F103C8T6 最小系统板 / blue pill）：
 *   - LED:  PC13（板载，低电平点亮，见 board_led_on/off 实现注释）
 *   - SC7A20 模块经 I2C1: SCL -> PB6, SDA -> PB7, VCC -> 3V3, GND -> GND
 *   - 系统时钟: 外部 8MHz HSE，PLL x9 -> 72MHz（板子必须带 8MHz 晶振）
 */
#ifndef BSP_BOARD_H
#define BSP_BOARD_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 板级初始化：HAL、72MHz 时钟、LED、DWT 微秒计数、I2C1。 */
void board_init(void);

/** LED 点亮（PC13 低电平点亮）。 */
void board_led_on(void);
/** LED 熄灭。 */
void board_led_off(void);
/** LED 翻转。 */
void board_led_toggle(void);

/** 空闲处理，进入 WFI 等待下一次 SysTick 唤醒。 */
void board_idle(void);

/** 出错处理：关闭中断并停住，便于调试器定位。 */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BOARD_H */
