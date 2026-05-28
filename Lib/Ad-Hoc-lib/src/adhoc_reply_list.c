/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_reply_list.c
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            组网确认队列实现
**                          环形 FIFO 确认队列, 按 node_id 去重, 用于 A 帧 flag=7 入网确认打包
**--------------------------------------------------------------------------------------------------------
*/

#include "adhoc_reply_list.h"

#include <string.h>

/**
 * @brief 检查队列中是否已包含指定 node_id
 */
static int adhoc_reply_list_contains(const adhoc_reply_list_t *list, uint32_t node_id)
{
    uint8_t index;
    uint8_t pos;

    if (list == 0)
    {
        return 0;
    }
    for (index = 0u; index < list->count; ++index)
    {
        pos = (uint8_t)((list->head + index) % ADHOC_REPLY_LIST_CAPACITY);
        if (list->ids[pos].node_id == node_id)
        {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief 复位确认队列(清空所有条目)
 */
void adhoc_reply_list_reset(adhoc_reply_list_t *list)
{
    if (list == 0)
    {
        return;
    }
    memset(list, 0, sizeof(*list));
}

/**
 * @brief 推入唯一 ID: 已存在则更新 id_flag, 不存在则追加
 * @param list 队列实例
 * @param id   载荷 ID (id_flag + node_id)
 * @return 1成功 0失败(队列满或参数非法)
 */
int adhoc_reply_list_push_unique(adhoc_reply_list_t *list, adhoc_payload_id_t id)
{
    uint8_t index;
    uint8_t pos;

    if (list == 0 || id.node_id == 0u || id.node_id > ADHOC_SENDER_NODE_MAX)
    {
        return 0;
    }
    if (id.id_flag > ADHOC_PAYLOAD_ID_FLAG_MAX)
    {
        return 0;
    }
    if (adhoc_reply_list_contains(list, id.node_id))
    {
        for (index = 0u; index < list->count; ++index)
        {
            pos = (uint8_t)((list->head + index) % ADHOC_REPLY_LIST_CAPACITY);
            if (list->ids[pos].node_id == id.node_id)
            {
                list->ids[pos].id_flag = id.id_flag;
                return 1;
            }
        }
        return 1;
    }
    if (list->count >= ADHOC_REPLY_LIST_CAPACITY)
    {
        return 0;
    }

    list->ids[list->tail] = id;
    list->tail = (uint8_t)((list->tail + 1u) % ADHOC_REPLY_LIST_CAPACITY);
    list->count++;
    return 1;
}

/**
 * @brief 推入入网确认条目(自动设置 id_flag=7)
 */
int adhoc_reply_list_push_confirm_unique(adhoc_reply_list_t *list, uint32_t node_id)
{
    adhoc_payload_id_t id;

    if (list == 0 || node_id == 0u || node_id > ADHOC_SENDER_NODE_MAX)
    {
        return 0;
    }
    id.id_flag = ADHOC_PAYLOAD_ID_FLAG_JOIN_CONFIRM;
    id.node_id = node_id;
    return adhoc_reply_list_push_unique(list, id);
}

/**
 * @brief 从队头弹出一个 ID
 */
int adhoc_reply_list_pop(adhoc_reply_list_t *list, adhoc_payload_id_t *out_id)
{
    adhoc_payload_id_t id;

    if (list == 0 || out_id == 0 || list->count == 0u)
    {
        return 0;
    }

    id = list->ids[list->head];
    memset(&list->ids[list->head], 0, sizeof(list->ids[list->head]));
    list->head = (uint8_t)((list->head + 1u) % ADHOC_REPLY_LIST_CAPACITY);
    list->count--;
    *out_id = id;
    return 1;
}

/**
 * @brief 获取队列当前大小
 */
uint8_t adhoc_reply_list_size(const adhoc_reply_list_t *list)
{
    return list == 0 ? 0u : list->count;
}

/**
 * @brief 构建 A 帧确认区: 从队列弹出条目, 统一标记为 flag=7, 打包到 content
 * @param content_offset 写入起始偏移(LMT_A 之后)
 * @param max_ids        最大确认条目数
 * @param out_used_ids   输出实际写入条目数(可为NULL)
 */
int adhoc_reply_list_build_confirm_payload(adhoc_reply_list_t *list, uint8_t content[ADHOC_FRAME_CONTENT_LEN],
                                           uint8_t content_offset, uint8_t max_ids, uint8_t *out_used_ids)
{
    uint8_t used_ids = 0u;
    uint8_t limit;
    uint8_t capacity;
    adhoc_payload_id_t packed_id;

    if (list == 0 || content == 0)
    {
        return 0;
    }
    if (content_offset >= ADHOC_FRAME_CONTENT_LEN)
    {
        return 0;
    }

    capacity = (uint8_t)((ADHOC_FRAME_CONTENT_LEN - content_offset) / ADHOC_REPLY_ITEM_ENCODED_LEN);
    limit = max_ids > ADHOC_REPLY_MAX_PER_FRAME ? ADHOC_REPLY_MAX_PER_FRAME : max_ids;
    if (limit > capacity)
    {
        limit = capacity;
    }
    while (used_ids < limit)
    {
        if (!adhoc_reply_list_pop(list, &packed_id))
        {
            break;
        }
        packed_id.id_flag = ADHOC_PAYLOAD_ID_FLAG_JOIN_CONFIRM;
        if (!adhoc_payload_id_pack(packed_id, &content[content_offset + used_ids * ADHOC_REPLY_ITEM_ENCODED_LEN]))
        {
            return 0;
        }
        used_ids++;
    }

    if (out_used_ids != 0)
    {
        *out_used_ids = used_ids;
    }
    return 1;
}
