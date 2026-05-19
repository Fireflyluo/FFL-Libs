/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_data_plane.h
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            数据面模块
**                          负责 D 帧编解码、去重表、待发队列、ACK 队列、TX 报告队列、数据转发逻辑
**--------------------------------------------------------------------------------------------------------
*/

#ifndef ADHOC_DATA_PLANE_H
#define ADHOC_DATA_PLANE_H

#include "adhoc_config.h"
#include "adhoc_frame.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 数据消息用户字段长度 */
#define ADHOC_DATA_MSG_USER_LEN 17u
/** @brief 每帧最大 ACK 条目数 */
#define ADHOC_DATA_ACK_MAX_PER_FRAME 3u
/** @brief 去重表容量 */
#define ADHOC_DATA_DEDUP_CAPACITY ADHOC_CONFIG_DATA_DEDUP_CAPACITY
/** @brief ACK 队列容量 */
#define ADHOC_DATA_ACK_QUEUE_CAPACITY ADHOC_CONFIG_DATA_ACK_QUEUE_CAPACITY
/** @brief 数据待发队列容量 */
#define ADHOC_DATA_TX_QUEUE_CAPACITY ADHOC_CONFIG_DATA_TX_QUEUE_CAPACITY
/** @brief TX 报告队列容量 */
#define ADHOC_DATA_TX_REPORT_QUEUE_CAPACITY ADHOC_CONFIG_DATA_TX_REPORT_QUEUE_CAPACITY
/** @brief 数据重试上限 */
#define ADHOC_DATA_TX_RETRY_MAX ADHOC_CONFIG_DATA_TX_RETRY_MAX
/** @brief 默认去重窗口(6小时, ms) */
#define ADHOC_DATA_DEFAULT_DEDUP_WINDOW_MS ADHOC_CONFIG_DATA_DEFAULT_DEDUP_WINDOW_MS
/** @brief 网关 ID 最大值 */
#define ADHOC_DATA_GATEWAY_ID_MAX ADHOC_CONFIG_DATA_GATEWAY_ID_MAX
/** @brief LMT_D 最大值(24bit) */
#define ADHOC_DATA_LMT_D_MAX 0x00FFFFFFu
/** @brief 直接下级绑定记录容量 */
#define ADHOC_DATA_DOWNSTREAM_BINDING_CAPACITY ADHOC_CONFIG_SM_DOWNSTREAM_BINDING_CAPACITY

/** @brief 数据来源类型 */
typedef enum {
    ADHOC_DATA_ORIGIN_UNKNOWN = 0u, /**< 未知 */
    ADHOC_DATA_ORIGIN_SOURCE  = 1u, /**< 源发数据(sender==source) */
    ADHOC_DATA_ORIGIN_FORWARD = 2u  /**< 转发数据(sender≠source) */
} adhoc_data_origin_t;

/** @brief 数据帧接收处理结果 */
typedef enum {
    ADHOC_DATA_RX_IGNORED   = 0u, /**< 忽略 */
    ADHOC_DATA_RX_ACCEPTED  = 1u, /**< 接受(新数据) */
    ADHOC_DATA_RX_DUPLICATE = 2u, /**< 重复 */
    ADHOC_DATA_RX_ACK       = 3u  /**< ACK帧 */
} adhoc_data_rx_result_t;

/**
 * @brief 数据消息
 */
typedef struct
{
    adhoc_payload_id_t source;             /**< 数据源标识 */
    uint32_t lmt_d;                        /**< 数据时间戳 LMT_D(24bit) */
    uint8_t user[ADHOC_DATA_MSG_USER_LEN]; /**< 用户数据 */
} adhoc_data_msg_t;

/**
 * @brief ACK 条目
 */
typedef struct
{
    adhoc_payload_id_t source; /**< 被确认的数据源 */
    uint32_t lmt_d;            /**< 被确认的 LMT_D */
} adhoc_data_ack_item_t;

/** @brief TX 报告码 */
typedef enum {
    ADHOC_DATA_TX_REPORT_NONE            = 0u, /**< 无 */
    ADHOC_DATA_TX_REPORT_ACKED           = 1u, /**< 确认 */
    ADHOC_DATA_TX_REPORT_RETRY_EXHAUSTED = 2u  /**< 重试耗尽 */
} adhoc_data_tx_report_code_t;

/**
 * @brief TX 报告
 */
typedef struct
{
    uint8_t code;               /**< 报告码 */
    adhoc_data_ack_item_t item; /**< 关联的数据标识 */
    uint8_t retry_count;        /**< 重试次数 */
} adhoc_data_tx_report_t;

/**
 * @brief 数据面使用的直接下级绑定记录
 */
typedef struct
{
    uint8_t  used;           /**< 是否占用 */
    uint8_t  child_bind_no;  /**< 发送者绑定到本节点时使用的编号 */
    uint8_t  child_level;    /**< 发送者级别 */
    uint8_t  gateway_no;     /**< 所属网关编号 */
    uint32_t child_id;       /**< 子节点ID */
    uint32_t last_seen_us;   /**< 最近观测时刻(μs) */
} adhoc_data_downstream_binding_t;

/**
 * @brief 数据面配置
 */
typedef struct
{
    uint16_t domain_id;       /**< 域号 */
    uint32_t node_id;         /**< 节点ID */
    uint8_t gateway_no;       /**< 网关编号 */
    uint8_t role_gateway;     /**< 网关角色标记 */
    uint32_t t5_us;           /**< 应答周期 */
    uint32_t dedup_window_ms; /**< 去重窗口(ms) */
    uint8_t tx_retry_max;     /**< 重试上限 */
} adhoc_data_plane_cfg_t;

/** @brief 去重表条目 */
typedef struct
{
    uint8_t used;              /**< 是否占用 */
    adhoc_payload_id_t source; /**< 数据源 */
    uint32_t lmt_d;            /**< LMT_D */
    uint32_t last_seen_ms;     /**< 最近观测时间(ms) */
} adhoc_data_dedup_entry_t;

/** @brief ACK 队列条目 */
typedef struct
{
    uint8_t used;               /**< 是否占用 */
    adhoc_data_ack_item_t item; /**< ACK条目 */
} adhoc_data_ack_queue_entry_t;

/** @brief 待发队列条目 */
typedef struct
{
    uint8_t used;         /**< 是否占用 */
    adhoc_data_msg_t msg; /**< 数据消息 */
    uint8_t level;        /**< 发送级别 */
    uint8_t gateway_no;   /**< 网关编号 */
    uint8_t retry_count;  /**< 已重试次数 */
    uint32_t next_tx_us;  /**< 下次发送时刻(μs) */
} adhoc_data_tx_entry_t;

/** @brief TX 报告队列条目 */
typedef struct
{
    uint8_t used;                  /**< 是否占用 */
    adhoc_data_tx_report_t report; /**< 报告内容 */
} adhoc_data_tx_report_entry_t;

/**
 * @brief 数据面实例(全部静态内存)
 */
typedef struct
{
    uint8_t inited;                                                                    /**< 是否已初始化 */
    adhoc_data_plane_cfg_t cfg;                                                        /**< 配置 */
    uint8_t role_gateway;                                                              /**< 网关角色标记 */
    adhoc_data_dedup_entry_t dedup[ADHOC_DATA_DEDUP_CAPACITY];                         /**< 去重表 */
    adhoc_data_ack_queue_entry_t ack_queue[ADHOC_DATA_ACK_QUEUE_CAPACITY];             /**< ACK队列 */
    uint8_t ack_head;                                                                  /**< ACK队列头 */
    uint8_t ack_tail;                                                                  /**< ACK队列尾 */
    uint8_t ack_count;                                                                 /**< ACK队列计数 */
    uint32_t next_ack_tx_us;                                                           /**< 下次ACK发送时刻 */
    adhoc_data_tx_entry_t tx_queue[ADHOC_DATA_TX_QUEUE_CAPACITY];                      /**< 待发队列 */
    adhoc_data_tx_report_entry_t tx_report_queue[ADHOC_DATA_TX_REPORT_QUEUE_CAPACITY]; /**< 报告队列 */
    uint8_t tx_report_head;                                                            /**< 报告队列头 */
    uint8_t tx_report_tail;                                                            /**< 报告队列尾 */
    uint8_t tx_report_count;                                                           /**< 报告队列计数 */
    uint8_t joined_level;                                                              /**< 入网级别 */
    uint32_t upstream_id;                                                              /**< 当前上级ID */
    uint8_t upstream_no;                                                               /**< 上级绑定编号 */
    uint8_t upstream_gateway_no;                                                       /**< 上级网关编号 */
    uint8_t downstream_binding_count;                                                  /**< 直接下级绑定记录数 */
    adhoc_data_downstream_binding_t downstream_bindings[ADHOC_DATA_DOWNSTREAM_BINDING_CAPACITY]; /**< 直接下级绑定表 */
    uint8_t has_last_rx_data;                                                          /**< 是否有最近接收数据 */
    adhoc_data_msg_t last_rx_data;                                                     /**< 最近接收的数据消息 */
    adhoc_data_origin_t last_rx_origin;                                                /**< 最近接收数据来源 */
    uint8_t last_rx_duplicate;                                                         /**< 最近接收是否重复 */
    uint8_t last_ack_hit_count;                                                        /**< 最近接收ACK命中计数 */
    uint8_t last_rx_ack_count;                                                         /**< 最近接收ACK条目数 */
    adhoc_data_ack_item_t last_rx_acks[ADHOC_DATA_ACK_MAX_PER_FRAME];                  /**< 最近接收ACK列表 */
} adhoc_data_plane_t;

/* ========== D 帧/ACK 编解码 ========== */

int adhoc_data_msg_pack(const adhoc_data_msg_t *msg, uint8_t content_out[ADHOC_FRAME_CONTENT_LEN]);
int adhoc_data_msg_unpack(const uint8_t content[ADHOC_FRAME_CONTENT_LEN], adhoc_data_msg_t *msg_out);
int adhoc_data_ack_payload_pack(const adhoc_data_ack_item_t *items, uint8_t item_count,
                                uint8_t content_out[ADHOC_FRAME_CONTENT_LEN]);
int adhoc_data_ack_payload_unpack(const uint8_t content[ADHOC_FRAME_CONTENT_LEN],
                                  adhoc_data_ack_item_t items_out[ADHOC_DATA_ACK_MAX_PER_FRAME],
                                  uint8_t *item_count_out);

/* ========== 数据面 API ========== */

int adhoc_data_plane_init(adhoc_data_plane_t *plane, const adhoc_data_plane_cfg_t *cfg);
int adhoc_data_plane_set_role(adhoc_data_plane_t *plane, uint8_t role_gateway);
int adhoc_data_plane_set_route(adhoc_data_plane_t *plane, uint8_t joined_level, uint8_t upstream_gateway_no,
                               uint8_t upstream_no, uint32_t upstream_id,
                               const adhoc_data_downstream_binding_t *downstream_bindings,
                               uint8_t downstream_binding_count);
void adhoc_data_plane_reset(adhoc_data_plane_t *plane);
int adhoc_data_plane_submit_source_data(adhoc_data_plane_t *plane, uint8_t source_id_flag, uint32_t lmt_d,
                                        const uint8_t user[ADHOC_DATA_MSG_USER_LEN], uint8_t level, uint8_t gateway_no,
                                        uint32_t now_us);
int adhoc_data_plane_pop_tx_report(adhoc_data_plane_t *plane, adhoc_data_tx_report_t *out_report);
adhoc_data_rx_result_t adhoc_data_plane_on_rx(adhoc_data_plane_t *plane, const adhoc_frame_fields_t *fields, uint32_t ts_us);
int adhoc_data_plane_poll_gateway_ack(adhoc_data_plane_t *plane, uint32_t now_us, adhoc_frame_fields_t *out_fields, uint8_t *out_has_tx);
int adhoc_data_plane_poll_forward_tx(adhoc_data_plane_t *plane, uint32_t now_us, adhoc_frame_fields_t *out_fields, uint8_t *out_has_tx);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_DATA_PLANE_H */
