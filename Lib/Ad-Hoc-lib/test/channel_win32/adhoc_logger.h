/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_logger.h
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            仿真日志系统
**                          仿嵌入式串口日志, 每节点独立文件输出, slot 索引隔离,
**                          CriticalSection 保护, 最多 64 个日志槽
**--------------------------------------------------------------------------------------------------------
*/

#ifndef ADHOC_LOGGER_H
#define ADHOC_LOGGER_H

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 最大日志槽数 */
#define ADHOC_LOGGER_MAX_SLOTS 64

/**
 * @brief 初始化日志系统(全局单例)
 * @param logs_dir 日志输出目录路径
 */
void adhoc_logger_init(const char *logs_dir);

/**
 * @brief 为指定槽位打开日志文件
 * @param slot_idx  槽位索引(0~63)
 * @param file_name 文件名(不含扩展名, 自动追加 .log)
 */
void adhoc_logger_open_slot(uint8_t slot_idx, const char *file_name);

/**
 * @brief 关闭指定槽位的日志文件
 */
void adhoc_logger_close_slot(uint8_t slot_idx);

/**
 * @brief 向指定槽位写入格式化日志(自动加时间戳和换行)
 * @param slot_idx 槽位索引
 * @param format   printf 格式字符串
 */
void adhoc_logger_log(uint8_t slot_idx, const char *format, ...);

/**
 * @brief 关闭所有日志文件并释放资源
 */
void adhoc_logger_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_LOGGER_H */
