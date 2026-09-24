/**
 * @file w25q_hw.h
 * @brief W25Q 硬件 SPI1 重映射 + DMA/中断传输（PB3/4/5，CS=PA15）。
 *
 * 该接口用于与 GPIO 软件 SPI 做独立对照。正式硬件事务由 DMA 完成中断收尾；
 * 另保留轮询、寄存器和硬件 NSS 的只读诊断入口。所有路径都不会回退到软件 SPI，
 * 以免掩盖 SPI1 重映射路径的问题。
 */
#ifndef W25Q_HW_H
#define W25Q_HW_H

#include <stdint.h>

#include "w25q.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化 SPI1 remap、DMA1 CH2/CH3 与 NVIC；可重复调用。 */
int w25q_hw_init(void);

/** 硬件 SPI DMA 读 JEDEC，用于确认 remap 通路。 */
int w25q_jedec_hw(uint8_t id[3]);
/** 同一 SPI1 配置的轮询 JEDEC/读，用于与 DMA 路径对照。 */
int w25q_jedec_hw_polling(uint8_t id[3]);
int w25q_read_hw_polling(uint32_t addr, uint8_t *data, uint16_t len);
/** 直接访问 SPI1 数据寄存器的 JEDEC 对照，不经过 HAL 传输函数。 */
int w25q_jedec_hw_register(uint8_t id[3]);
/** SPI1 硬件 NSS（PA15）自动片选的 JEDEC 对照。 */
int w25q_jedec_hw_hardnss(uint8_t id[3]);

/** DMA 读、4KB 擦除、页写与验证页写。 */
int w25q_read_hw(uint32_t addr, uint8_t *data, uint16_t len);
int w25q_read_status_hw(uint8_t *status);
int w25q_erase_sector_hw(uint32_t addr);
int w25q_write_page_hw(uint32_t addr, const uint8_t *data, uint16_t len);
int w25q_write_page_verified_hw(uint32_t addr, const uint8_t *data,
                                 uint16_t len);

/** 返回最近一次硬件 SPI 页写的状态快照。 */
void w25q_hw_get_last_write_diag(w25q_write_diag_t *out);

/** 返回 DMA 完成与错误回调次数，证明传输由中断收尾。 */
void w25q_hw_get_dma_stats(uint32_t *completed, uint32_t *errors);

/** 调试：读 RCC_APB2ENR（AFIO 时钟 bit0）与 AFIO_MAPR。 */
void w25q_hw_dump_afio(uint32_t *apb2enr, uint32_t *mapr);

/** 调试：读 SPI1 CR1/SR。 */
void w25q_hw_dump_spi(uint32_t *cr1, uint32_t *sr);

/** 调试：读取 PA15 与 PB3/PB4/PB5 的 GPIO 配置寄存器。 */
void w25q_hw_dump_gpio(uint32_t *gpioa_crh, uint32_t *gpiob_crl);

/** DMA 读并返回耗时微秒（不含首次初始化）。 */
uint32_t w25q_read_hw_timed(uint32_t addr, uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
