/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_crc8.h
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            CRC8 校验模块
**                          固定参数 CRC8 计算与帧校验 (poly=0x07, init=0x00, xorout=0x00)
**--------------------------------------------------------------------------------------------------------
*/

#ifndef ADHOC_CRC8_H
#define ADHOC_CRC8_H

#include "adhoc_config.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief CRC8 多项式 */
#define ADHOC_CRC8_POLY 0x07u
/** @brief CRC8 初始值 */
#define ADHOC_CRC8_INIT 0x00u
/** @brief CRC8 输出异或值 */
#define ADHOC_CRC8_XOROUT 0x00u

/**
 * @brief CRC8 查表的存储段属性（可由工程侧覆盖）
 *
 * - 默认：RISC-V + GCC 下放入 `.flash1_rodata`
 * - 其他平台：默认无特殊段属性
 */
#ifndef ADHOC_CRC8_TABLE_STORAGE
#define ADHOC_CRC8_TABLE_STORAGE ADHOC_CONFIG_CRC8_TABLE_STORAGE
#endif

/**
 * @brief 计算任意数据的 CRC8
 * @param data 数据指针
 * @param len  数据长度(字节)
 * @return CRC8 值
 */
uint8_t adhoc_crc8_compute(const uint8_t *data, uint16_t len);

/**
 * @brief 计算32B帧的 CRC8(覆盖前31字节)
 * @param frame 32字节帧缓冲区
 * @return CRC8 值
 */
uint8_t adhoc_crc8_frame(const uint8_t frame[32]);

/**
 * @brief 验证32B帧的 CRC8
 * @param frame 32字节帧缓冲区
 * @return 1校验通过 0失败
 */
int adhoc_crc8_verify_frame(const uint8_t frame[32]);

/**
 * @brief 计算并写入 CRC8 到帧的最后一个字节
 * @param frame 32字节帧缓冲区(第31字节将被覆盖)
 */
void adhoc_crc8_write_frame(uint8_t frame[32]);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_CRC8_H */
