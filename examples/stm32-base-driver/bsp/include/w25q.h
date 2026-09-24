/**
 * @file w25q.h
 * @brief W25Qxx **软件 SPI**（不占硬件 SPI1，与 ST7735 无冲突）。
 *
 * 引脚（GPIO）：PA15=CS, PB3=SCK, PB5=MOSI, PB4=MISO
 * 需 SWJ_NOJTAG 释放 PA15/PB3/PB4（保留 SWD）。
 */
#ifndef W25Q_H
#define W25Q_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define W25Q_CMD_WREN 0x06
#define W25Q_CMD_RDSR1 0x05
#define W25Q_CMD_READ 0x03
#define W25Q_CMD_PP 0x02
#define W25Q_CMD_SE 0x20
#define W25Q_CMD_JEDEC 0x9F

typedef struct {
  uint8_t status_after_wren;
  uint8_t status_after_command;
  uint8_t status_after_wait;
  uint8_t saw_busy;
} w25q_write_diag_t;

/** GPIO + 关 JTAG；可选读 JEDEC ID。 */
int w25q_init(void);
int w25q_read_jedec(uint8_t id[3]);

/** 4KB 扇区擦除 / 256B 页编程 / 任意长度读（软 SPI）。
 * 页编程要求数据不跨 256B 页边界；成功仅表示 PP 命令已完成，
 * 不包含回读校验。失败返回负 errno 风格错误码。 */
int w25q_erase_sector(uint32_t addr);
int w25q_write_page(uint32_t addr, const uint8_t *data, uint16_t len);
/** 页编程后回读校验；数据不一致返回 -EIO。 */
int w25q_write_page_verified(uint32_t addr, const uint8_t *data,
                              uint16_t len);
int w25q_read(uint32_t addr, uint8_t *data, uint16_t len);

/** 返回最近一次 w25q_write_page() 的状态快照，供板端定位 PP 是否被芯片接受。 */
void w25q_get_last_write_diag(w25q_write_diag_t *out);

/**
 * 把整幅 RGB565 写入 flash（自动按页拆分）。
 * 会擦除每个覆盖到的完整 4KB 扇区，并逐页回读校验；调用方必须保证目标图像槽独占且扇区对齐。
 * @return 0 表示每页已回读匹配；失败返回负 errno 风格错误码
 */
int w25q_write_image(uint32_t addr, const uint8_t *data, uint32_t len);

/** 软件 SPI 连续读 len 字节，返回耗时微秒（DWT）。 */
uint32_t w25q_read_timed(uint32_t addr, uint8_t *data, uint16_t len);

/** 读状态寄存器1（RDSR1）。 */
uint8_t w25q_read_status(void);

/** 写状态寄存器1（WREN+WRSR），用于清 BP 保护位。 */
int w25q_write_status(uint8_t sr1);

#ifdef __cplusplus
}
#endif

#endif
