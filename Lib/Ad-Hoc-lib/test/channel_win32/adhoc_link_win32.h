/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_link_win32.h
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            Windows 链路适配层
**                          实现 adhoc_link_ops_t 六接口, 桥接协议层与空域信道,
**                          提供 virtual_time / channel / logger 的集成上下文
**--------------------------------------------------------------------------------------------------------
*/

#ifndef ADHOC_LINK_WIN32_H
#define ADHOC_LINK_WIN32_H

#include "adhoc_channel.h"
#include "adhoc_virtual_time.h"

#include "adhoc_link.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Windows 链路适配上下文(透传给 adhoc_link_ops_t 的各函数)
 */
typedef struct
{
    adhoc_channel_t      *channel;   /**< 指向共享空域信道 */
    uint8_t               node_idx;  /**< 本节点在信道中的索引 */
    uint8_t               log_slot;  /**< 日志槽索引 */
    adhoc_virtual_time_t *vt;        /**< 虚拟时间实例 */
    uint32_t              node_id;   /**< 协议层节点 ID */
    uint32_t              rand_state;/**< 随机数状态 */
} adhoc_link_win32_ctx_t;

/** @brief 全局 Windows 链路操作接口实例 */
extern const adhoc_link_ops_t g_adhoc_link_win32_ops;

/**
 * @brief 初始化链路适配上下文
 * @param ctx      上下文实例
 * @param channel  空域信道指针
 * @param node_idx 本节点在信道中的索引
 * @param log_slot 日志槽索引
 * @param vt       虚拟时间指针
 * @param node_id  协议节点 ID
 */
void adhoc_link_win32_ctx_init(adhoc_link_win32_ctx_t *ctx,
                               adhoc_channel_t *channel, uint8_t node_idx,
                               uint8_t log_slot,
                               adhoc_virtual_time_t *vt,
                               uint32_t node_id);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_LINK_WIN32_H */
