/**
 * @file usb_img.h
 * @brief USB CDC 分包收图写入 W25Q；日志走 USART1(COM4)。
 *
 * 主机协议（二进制、小端、单包最大 64B）：
 *   请求: "IP" | ver=1 | type | seq:u16 | idx:u8 | len:u8 |
 *         offset:u32 | payload[len] | crc16-ccitt:u16。
 *   type=1 BEGIN（payload=total_len:u32），type=2 DATA（最多 48B），
 *   type=3 END，type=4 ABORT。
 *   回应: "IA" | ver=1 | ACK/NACK | seq:u16 | status:u8 | idx:u8 |
 *         committed_offset:u32。
 * 主机一次只发送一个包；DATA 的 ACK 在页编程完成后发送。
 */
#ifndef USB_IMG_H
#define USB_IMG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化 CherryUSB CDC（PA11/12，48MHz）。 */
void usb_img_init(void);

/** 供主循环调用：处理一个 USB 包、写入 W25Q 并回复 ACK。 */
void usb_img_poll(void);

uint8_t usb_img_ready_mask(void);

/** 正在擦/写 W25Q；lcd 应跳过本拍刷屏，避免抢软 SPI。 */
uint8_t usb_img_busy(void);

/** 上电探测后并入 ready_mask。 */
void usb_img_set_ready_mask(uint8_t mask);

/** 从 0x0F0000 魔数扇区恢复 ready_mask。 */
uint8_t usb_img_load_ready_from_flash(void);

#ifdef __cplusplus
}
#endif

#endif
