/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_reply_list.h
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            组网确认队列模块
**                          维护环形 FIFO 确认队列, 用于 A 帧的入网确认(flag=7)打包与发送
**--------------------------------------------------------------------------------------------------------
*/

#ifndef ADHOC_REPLY_LIST_H
#define ADHOC_REPLY_LIST_H

#include "adhoc_config.h"
#include "adhoc_frame.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 确认队列容量 */
#define ADHOC_REPLY_LIST_CAPACITY ADHOC_CONFIG_REPLY_LIST_CAPACITY
/** @brief 每A帧最多确认数(flag=7) */
#define ADHOC_REPLY_MAX_PER_FRAME ADHOC_CONFIG_REPLY_MAX_PER_FRAME
/** @brief 每个ID编码长度(字节) */
#define ADHOC_REPLY_ITEM_ENCODED_LEN 4u
/** @brief A帧LMT_A字段长度 */
#define ADHOC_A_CONTENT_LMT_A_LEN 4u
/** @brief A帧确认区起始偏移(跳过LMT_A) */
#define ADHOC_A_CONFIRM_PAYLOAD_OFFSET ADHOC_A_CONTENT_LMT_A_LEN
/** @brief 确认区总字节数 */
#define ADHOC_A_CONFIRM_PAYLOAD_BYTES (ADHOC_REPLY_MAX_PER_FRAME * ADHOC_REPLY_ITEM_ENCODED_LEN)
/** @brief 入网请求头部字节数(一个ID) */
#define ADHOC_A_JOIN_HEAD_BYTES ADHOC_REPLY_ITEM_ENCODED_LEN

#if (ADHOC_A_CONFIRM_PAYLOAD_OFFSET + ADHOC_A_CONFIRM_PAYLOAD_BYTES) > ADHOC_FRAME_CONTENT_LEN
#error "Confirm payload exceeds content length (24)."
#endif

#if (ADHOC_A_CONFIRM_PAYLOAD_OFFSET + ADHOC_A_JOIN_HEAD_BYTES + ((ADHOC_REPLY_MAX_PER_FRAME - 1u) * ADHOC_REPLY_ITEM_ENCODED_LEN)) > ADHOC_FRAME_CONTENT_LEN
#error "Join response exceeds content length (24)."
#endif

/**
 * @brief 确认队列(环形 FIFO)
 */
typedef struct
{
    adhoc_payload_id_t ids[ADHOC_REPLY_LIST_CAPACITY]; /**< ID 数组 */
    uint8_t head;    /**< 队头索引 */
    uint8_t tail;    /**< 队尾索引 */
    uint8_t count;   /**< 当前计数 */
} adhoc_reply_list_t;

/**
 * @brief 复位确认队列
 * @param list 队列实例
 */
void adhoc_reply_list_reset(adhoc_reply_list_t *list);

/**
 * @brief 向队列推入唯一 ID(按 node_id 去重, 已存在则更新 id_flag)
 * @param list 队列实例
 * @param id   载荷 ID
 * @return 1成功 0失败(队列满)
 */
int adhoc_reply_list_push_unique(adhoc_reply_list_t *list, adhoc_payload_id_t id);

/**
 * @brief 推入入网确认条目(自动设置 flag=7)
 * @param list    队列实例
 * @param node_id 要确认的节点ID
 * @return 1成功 0失败
 */
int adhoc_reply_list_push_confirm_unique(adhoc_reply_list_t *list, uint32_t node_id);

/**
 * @brief 从队列弹出一个 ID
 * @param list   队列实例
 * @param out_id 输出 ID
 * @return 1成功 0队列空
 */
int adhoc_reply_list_pop(adhoc_reply_list_t *list, adhoc_payload_id_t *out_id);

/**
 * @brief 获取队列当前大小
 * @param list 队列实例
 * @return 当前条目数
 */
uint8_t adhoc_reply_list_size(const adhoc_reply_list_t *list);

/**
 * @brief 构建 A 帧确认区(从队列弹出并打包为 flag=7 条目)
 * @param list           队列实例
 * @param content        输出 content 缓冲区(24B)
 * @param content_offset 写入起始偏移
 * @param max_ids        最大确认条目数
 * @param out_used_ids   输出实际写入条目数
 * @return 1成功 0失败
 */
int adhoc_reply_list_build_confirm_payload(adhoc_reply_list_t *list, uint8_t content[ADHOC_FRAME_CONTENT_LEN],
                                           uint8_t content_offset, uint8_t max_ids, uint8_t *out_used_ids);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_REPLY_LIST_H */
