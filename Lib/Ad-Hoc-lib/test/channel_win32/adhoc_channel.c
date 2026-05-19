/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_channel.c
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            空域信道实现
**                          拓扑矩阵管理 + 环形 FIFO 帧分发,
**                          发送时根据拓扑矩阵将帧拷贝到所有可达节点的接收队列
**--------------------------------------------------------------------------------------------------------
*/

#include "adhoc_channel.h"

#include <string.h>

/**
 * @brief 初始化空域信道(含临界区)
 */
void adhoc_channel_init(adhoc_channel_t *ch, uint8_t max_nodes)
{
    if (ch == NULL || max_nodes == 0u || max_nodes > ADHOC_CHANNEL_MAX_NODES)
    {
        return;
    }

    memset(ch, 0, sizeof(*ch));
    ch->max_nodes = max_nodes;
    InitializeCriticalSection(&ch->lock);
}

/**
 * @brief 销毁空域信道(释放临界区)
 */
void adhoc_channel_deinit(adhoc_channel_t *ch)
{
    if (ch == NULL)
    {
        return;
    }
    DeleteCriticalSection(&ch->lock);
    memset(ch, 0, sizeof(*ch));
}

/**
 * @brief 设置单条链路 RSSI: rssi < -127 视为不可达
 */
void adhoc_channel_set_topo(adhoc_channel_t *ch, uint8_t from_idx, uint8_t to_idx, int8_t rssi)
{
    if (ch == NULL || from_idx >= ch->max_nodes || to_idx >= ch->max_nodes)
    {
        return;
    }

    EnterCriticalSection(&ch->lock);
    if (rssi < -127)
    {
        ch->topo[from_idx][to_idx] = 0u;
    }
    else
    {
        ch->topo[from_idx][to_idx] = (uint8_t)(rssi & 0xFF);
    }
    LeaveCriticalSection(&ch->lock);
}

/**
 * @brief 向信道发送帧: 遍历拓扑矩阵, 拷贝到所有可达节点(排除自身)的接收 FIFO
 */
int adhoc_channel_tx(adhoc_channel_t *ch, uint8_t from_idx, const uint8_t *frame, uint8_t len)
{
    uint8_t to_idx;
    adhoc_channel_node_rx_t *rx;
    int8_t rssi;

    if (ch == NULL || frame == NULL || len != ADHOC_FRAME_LEN || from_idx >= ch->max_nodes)
    {
        return 0;
    }

    EnterCriticalSection(&ch->lock);

    for (to_idx = 0u; to_idx < ch->max_nodes; ++to_idx)
    {
        if (to_idx == from_idx)
        {
            continue;
        }

        rssi = (int8_t)ch->topo[from_idx][to_idx];
        if (rssi == 0)
        {
            continue;
        }

        rx = &ch->nodes[to_idx];
        if (rx->count >= ADHOC_CHANNEL_RX_FIFO_SIZE)
        {
            continue;
        }

        memcpy(rx->fifo[rx->tail].frame, frame, ADHOC_FRAME_LEN);
        rx->fifo[rx->tail].len  = len;
        rx->fifo[rx->tail].rssi = rssi;
        rx->fifo[rx->tail].used = 1u;
        rx->tail = (uint16_t)((rx->tail + 1u) % ADHOC_CHANNEL_RX_FIFO_SIZE);
        rx->count++;
    }

    LeaveCriticalSection(&ch->lock);
    return 1;
}

/**
 * @brief 从指定节点接收队列取出队头帧
 * @return 1有帧(输出 frame/len/rssi), 0队列空
 */
int adhoc_channel_poll_rx(adhoc_channel_t *ch, uint8_t node_idx, uint8_t *frame_out, uint8_t *len_out, int8_t *rssi_out)
{
    adhoc_channel_node_rx_t *rx;
    adhoc_channel_rx_entry_t *entry;

    if (ch == NULL || frame_out == NULL || node_idx >= ch->max_nodes)
    {
        return 0;
    }

    EnterCriticalSection(&ch->lock);

    rx = &ch->nodes[node_idx];
    if (rx->count == 0u)
    {
        LeaveCriticalSection(&ch->lock);
        return 0;
    }

    entry = &rx->fifo[rx->head];
    if (entry->used == 0u)
    {
        LeaveCriticalSection(&ch->lock);
        return 0;
    }

    memcpy(frame_out, entry->frame, ADHOC_FRAME_LEN);
    if (len_out != NULL)
    {
        *len_out = entry->len;
    }
    if (rssi_out != NULL)
    {
        *rssi_out = entry->rssi;
    }

    memset(entry, 0, sizeof(*entry));
    rx->head = (uint16_t)((rx->head + 1u) % ADHOC_CHANNEL_RX_FIFO_SIZE);
    rx->count--;

    LeaveCriticalSection(&ch->lock);
    return 1;
}

/**
 * @brief 清空指定节点的接收队列
 */
void adhoc_channel_reset_node_rx(adhoc_channel_t *ch, uint8_t node_idx)
{
    if (ch == NULL || node_idx >= ch->max_nodes)
    {
        return;
    }

    EnterCriticalSection(&ch->lock);
    memset(&ch->nodes[node_idx], 0, sizeof(ch->nodes[node_idx]));
    LeaveCriticalSection(&ch->lock);
}
