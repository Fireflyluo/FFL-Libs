/**
 * @file board.h
 * @brief 板级 API（示例自有，**不属于** ffl 组件，也**不是** ports/）。
 *
 * 仓库分层回顾：
 *   components/  通用逻辑
 *   ports/       MCU 南向契约实现（I2C xfer、临界区等）
 *   bsp/  ← 本目录  具体板子的引脚、时钟、LED、串口
 *
 * app/ 只调用这里的函数；换板时通常只需改 bsp/，组件与 ports 可复用。
 *
 * 接线（STM32-LORA + ST7789 12pin，详见 硬件映射表.md）：
 *   LED      PA0 高电平亮
 *   蜂鸣器   PB6，**低电平响**
 *   调试串口 USART1 PA9/PA10 → COM4 @115200（TXE 环 + 可选 DMA）
 *   I2C1     重映射 PB8/PB9 → SC7A20
 *   W25Q     GPIO 软件 SPI：PA15/PB3/PB4/PB5
 *   ST7789   SPI1+DMA1_CH3：PA5=SCL, PA7=SDA, PA4=CSX, PB0=RESX, PB1=DCX
 *   背光     LEDK 硬件已接地常亮（封装镜像问题，见映射表）
 *   USB      PA11/12 CDC FF55:5711（D+ 需 1.5k 上拉）
 *   时钟     HSE 16MHz / 2 * 9 = 72MHz
 */
#ifndef BSP_BOARD_H
#define BSP_BOARD_H

#include "stm32f1xx_hal.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 板级初始化：HAL、72MHz 时钟、LED、UART1、DWT、I2C1。 */
void board_init(void);

/** LED 点亮（PA0 高电平点亮）。 */
void board_led_on(void);
/** LED 熄灭。 */
void board_led_off(void);
/** LED 翻转。 */
void board_led_toggle(void);

/** 蜂鸣器（PB6，TIM4_CH1 PWM）。板载为有源蜂鸣时也可用 on/off 电平。 */
void board_buzzer_on(void);   /* 稳定音 ~1kHz 或保持响 */
void board_buzzer_off(void);  /* 静音（PB6 释放为高） */
/** 按频率驱鸣（Hz）；0 等价 off。用于防空警报扫频。 */
void board_buzzer_tone(uint16_t hz);

/** UART1 环形缓冲写入一字节（printf 后端）。 */
void board_uart_putc(int ch);
/** UART1 环形缓冲写入字符串。 */
void board_uart_write(const char *s);
/** UART1 环形缓冲写入；返回实际入队字节数（供 ulog tx_try）。 */
int board_uart_write_try(const char *data, int len);
/** 主动踢一次 USART1 TX（DMA/中断）。 */
void board_uart_kick_tx(void);
/** 非阻塞收一字节；无数据返回 -1。 */
int board_uart_getc(void);
/** 供底层收发使用的 USART1 句柄。 */
UART_HandleTypeDef *board_uart_handle(void);

/** 空闲处理，进入 WFI 等待下一次 SysTick 唤醒。 */
void board_idle(void);

/** 出错处理：关闭中断并停住，便于调试器定位。 */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BOARD_H */
