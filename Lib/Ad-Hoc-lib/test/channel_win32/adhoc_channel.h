/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_channel.h
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            空域信道模块(仿真无线广播介质)
**                          拓扑矩阵控制节点间可达性, 环形 FIFO 管理每节点接收队列,
**                          CriticalSection 保护多线程安全
**--------------------------------------------------------------------------------------------------------
*/

#ifndef ADHOC_CHANNEL_H
#define ADHOC_CHANNEL_H

#include <stdint.h>
#include <windows.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 信道最大节点数 */
#define ADHOC_CHANNEL_MAX_NODES      64u
/** @brief 每节点接收 FIFO 容量 */
#define ADHOC_CHANNEL_RX_FIFO_SIZE   256u
/** @brief 协议帧固定长度 */
#define ADHOC_FRAME_LEN              32u

/**
 * @brief 接收 FIFO 条目(一帧+元数据)
 */
typedef struct
{
    uint8_t  frame[ADHOC_FRAME_LEN]; /**< 帧数据(32B) */
    uint8_t  len;                    /**< 帧长度 */
    int8_t   rssi;                   /**< RSSI(dBm) */
    uint8_t  used;                   /**< 条目是否有效 */
} adhoc_channel_rx_entry_t;

/**
 * @brief 节点接收 FIFO(环形队列)
 */
typedef struct
{
    adhoc_channel_rx_entry_t fifo[ADHOC_CHANNEL_RX_FIFO_SIZE]; /**< 环形缓冲区 */
    uint16_t head;   /**< 队头索引 */
    uint16_t tail;   /**< 队尾索引 */
    uint16_t count;  /**< 当前帧数 */
} adhoc_channel_node_rx_t;

/**
 * @brief 空域信道实例
 */
typedef struct
{
    uint8_t  max_nodes;      /**< 最大节点数 */
    uint8_t  topo[ADHOC_CHANNEL_MAX_NODES][ADHOC_CHANNEL_MAX_NODES]; /**< 拓扑矩阵: topo[from][to]=RSSI */
    adhoc_channel_node_rx_t nodes[ADHOC_CHANNEL_MAX_NODES]; /**< 各节点接收队列 */
    CRITICAL_SECTION lock;   /**< 临界区保护 */
} adhoc_channel_t;

/**
 * @brief 初始化空域信道
 * @param ch        信道实例
 * @param max_nodes 最大节点数(≤64)
 */
void adhoc_channel_init(adhoc_channel_t *ch, uint8_t max_nodes);

/**
 * @brief 销毁空域信道(释放临界区)
 */
void adhoc_channel_deinit(adhoc_channel_t *ch);

/**
 * @brief 设置拓扑矩阵中的单条链路 RSSI
 * @param ch       信道实例
 * @param from_idx 发送节点索引
 * @param to_idx   接收节点索引
 * @param rssi     信号强度(dBm), 0=不可达, <-127 视为不可达
 */
void adhoc_channel_set_topo(adhoc_channel_t *ch, uint8_t from_idx, uint8_t to_idx, int8_t rssi);

/**
 * @brief 向信道发送帧(广播到所有可达节点)
 * @param ch       信道实例
 * @param from_idx 发送节点索引
 * @param frame    帧数据(32B)
 * @param len      帧长度
 * @return 1成功 0失败
 */
int  adhoc_channel_tx(adhoc_channel_t *ch, uint8_t from_idx, const uint8_t *frame, uint8_t len);

/**
 * @brief 从指定节点接收队列取帧
 * @param ch        信道实例
 * @param node_idx  节点索引
 * @param frame_out 输出帧数据(32B)
 * @param len_out   输出帧长度
 * @param rssi_out  输出 RSSI
 * @return 1有帧 0队列空
 */
int  adhoc_channel_poll_rx(adhoc_channel_t *ch, uint8_t node_idx, uint8_t *frame_out, uint8_t *len_out, int8_t *rssi_out);

/**
 * @brief 清空指定节点的接收队列
 */
void adhoc_channel_reset_node_rx(adhoc_channel_t *ch, uint8_t node_idx);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_CHANNEL_H */
