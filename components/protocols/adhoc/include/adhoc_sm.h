/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_sm.h
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            组网状态机模块
**                          管理 ST1/U1/UN/C1/CN 状态迁移、邻居缓存(Top-K)、上级绑定表、确认队列
**--------------------------------------------------------------------------------------------------------
*/

#ifndef ADHOC_SM_H
#define ADHOC_SM_H

#include "adhoc_config.h"
#include "adhoc_frame.h"
#include "adhoc_reply_list.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 网关 ID 最大值 */
#define ADHOC_SM_GATEWAY_ID_MAX ADHOC_CONFIG_SM_GATEWAY_ID_MAX
/** @brief 信标 ID 最小值 */
#define ADHOC_SM_BEACON_ID_MIN ADHOC_CONFIG_SM_BEACON_ID_MIN
/** @brief 默认入网重试上限 */
#define ADHOC_SM_DEFAULT_RETRY_MAX ADHOC_CONFIG_SM_DEFAULT_RETRY_MAX
/** @brief 默认组网时间窗(30s) */
#define ADHOC_SM_DEFAULT_NETWORK_WINDOW_US ADHOC_CONFIG_SM_DEFAULT_NETWORK_WINDOW_US
/** @brief 强信号 RSSI 阈值(dBm) */
#define ADHOC_SM_RSSI_STRONG_MIN_DBM ADHOC_CONFIG_SM_RSSI_STRONG_MIN_DBM
/** @brief 中信号 RSSI 阈值(dBm) */
#define ADHOC_SM_RSSI_MEDIUM_MIN_DBM ADHOC_CONFIG_SM_RSSI_MEDIUM_MIN_DBM
/** @brief 最大入网级别 */
#define ADHOC_SM_JOIN_LEVEL_MAX ADHOC_CONFIG_SM_JOIN_LEVEL_MAX
/** @brief 上级绑定表容量 */
#define ADHOC_SM_UPSTREAM_BINDING_CAPACITY ADHOC_CONFIG_SM_UPSTREAM_BINDING_CAPACITY
/** @brief 邻居缓存容量(Top-K) */
#define ADHOC_SM_NEIGHBOR_CACHE_CAPACITY ADHOC_CONFIG_SM_NEIGHBOR_CACHE_CAPACITY
/** @brief 上级丢失判定周期数(3*T5内未收到即回退) */
#define ADHOC_SM_UPSTREAM_LOSS_CYCLES ADHOC_CONFIG_SM_UPSTREAM_LOSS_CYCLES
/** @brief 直接下级绑定记录容量 */
#define ADHOC_SM_DOWNSTREAM_BINDING_CAPACITY ADHOC_CONFIG_SM_DOWNSTREAM_BINDING_CAPACITY

/** @brief 状态机状态枚举 */
typedef enum
{
    ADHOC_SM_STATE_ST0 = 0u, /**< 深度休眠 */
    ADHOC_SM_STATE_ST1 = 1u, /**< 等待组网(交替T3侦听/T4休眠) */
    ADHOC_SM_STATE_U1  = 2u, /**< 未确认1级(已选网关方向, 发送入网请求) */
    ADHOC_SM_STATE_C1  = 3u, /**< 已确认1级(收到网关确认, 广播本级别信标) */
    ADHOC_SM_STATE_UN  = 4u, /**< 未确认N级(已选上级方向, 发送入网请求) */
    ADHOC_SM_STATE_CN  = 5u  /**< 已确认N级(收到上级确认, 广播本级别信标) */
} adhoc_sm_state_t;

/** @brief 接收帧处理结果 */
typedef enum
{
    ADHOC_SM_RX_IGNORED      = 0, /**< 帧被忽略 */
    ADHOC_SM_RX_ACCEPTED     = 1, /**< 帧被接受(无状态变化) */
    ADHOC_SM_RX_STATE_CHANGED = 2 /**< 帧触发了状态迁移 */
} adhoc_sm_rx_result_t;

/** @brief 状态机角色 */
typedef enum
{
    ADHOC_SM_ROLE_BEACON  = 0u, /**< 信标 */
    ADHOC_SM_ROLE_GATEWAY = 1u  /**< 网关 */
} adhoc_sm_role_t;

/**
 * @brief 状态机配置
 */
typedef struct
{
    uint8_t  role;               /**< 角色 */
    uint16_t domain_id;          /**< 域号 */
    uint32_t node_id;            /**< 节点ID */
    uint8_t  gateway_no;         /**< 网关编号 */
    uint32_t t2_us;              /**< 接收窗口(μs) */
    uint32_t t5_us;              /**< 应答周期(T1+T2, μs) */
    uint8_t  retry_max;          /**< 重试上限 */
    uint32_t network_window_us;  /**< 组网时间窗(μs) */
    uint32_t regroup_interval_us;/**< 重组网周期(μs) */
} adhoc_sm_cfg_t;

/**
 * @brief 接收事件(传递给状态机)
 */
typedef struct
{
    uint8_t         msg_class;   /**< 消息类别 */
    uint8_t         gateway_no;  /**< 网关编号 */
    uint8_t         level;       /**< 级别 */
    adhoc_sender_t  sender;      /**< 发送者 */
    uint8_t         content[ADHOC_FRAME_CONTENT_LEN]; /**< 帧内容 */
    int8_t          rssi;        /**< RSSI(dBm) */
    uint32_t        ts_us;       /**< 接收时刻(μs) */
} adhoc_sm_rx_event_t;

/**
 * @brief 直接下级绑定记录
 */
typedef struct
{
    uint8_t  used;           /**< 是否占用 */
    uint8_t  child_bind_no;  /**< 子节点绑定到本节点时使用的编号 */
    uint8_t  child_level;    /**< 子节点级别 */
    uint8_t  gateway_no;     /**< 所属网关编号 */
    uint32_t child_id;       /**< 子节点ID */
    uint32_t last_seen_us;   /**< 最近观测时刻(μs) */
} adhoc_sm_downstream_binding_t;

/**
 * @brief 状态机快照(对外只读)
 */
typedef struct
{
    adhoc_sm_state_t state;
    uint8_t  joined_level;
    uint32_t upstream_id;
    uint8_t  upstream_no;
    uint8_t  upstream_gateway_no;
    uint8_t  retry_count;
    uint32_t upstream_last_seen_us;
    uint8_t  gateway_network_started;
    uint8_t  gateway_network_locked;
    uint32_t gateway_network_start_us;
    uint32_t gateway_network_end_us;
    uint8_t  network_lock_active;
    uint8_t  network_lock_closed;
    uint32_t network_lock_end_us;
    uint8_t  downstream_binding_count;
    adhoc_sm_downstream_binding_t downstream_bindings[ADHOC_SM_DOWNSTREAM_BINDING_CAPACITY];
} adhoc_sm_snapshot_t;

/**
 * @brief 候选邻居(用于Top-K评估)
 */
typedef struct
{
    uint8_t  active;              /**< 是否有效 */
    uint8_t  upstream_level;      /**< 上级级别 */
    uint8_t  upstream_no;         /**< 上级绑定编号 */
    uint8_t  gateway_no;          /**< 网关编号 */
    uint32_t upstream_id;         /**< 上级节点ID */
    uint8_t  signal_rank;         /**< 信号等级(1弱/2中/3强) */
    uint8_t  required_hits;       /**< 所需命中次数 */
    uint8_t  window_cycles;       /**< 评估窗口(T5周期数) */
    uint8_t  success_hits;        /**< 已命中次数 */
    int8_t   last_rssi;           /**< 最近RSSI */
    uint32_t first_cycle_idx;     /**< 首次命中周期索引 */
    uint32_t last_hit_cycle_idx;  /**< 最近命中周期索引 */
    uint32_t last_seen_us;        /**< 最近观测时刻(μs) */
    uint32_t network_end_us;      /**< 该上级网络窗口结束时刻(μs) */
    uint8_t  has_last_hit_cycle;  /**< 是否有命中记录 */
} adhoc_sm_candidate_t;

/**
 * @brief 上级绑定表条目
 */
typedef struct
{
    uint8_t  used;           /**< 是否占用 */
    uint8_t  upstream_no;    /**< 绑定编号(0~5) */
    uint8_t  gateway_no;     /**< 网关编号 */
    uint8_t  upstream_level; /**< 上级级别 */
    uint32_t upstream_id;    /**< 上级节点ID */
} adhoc_sm_upstream_binding_t;

/**
 * @brief 组网状态机实例(全部静态内存)
 */
typedef struct
{
    uint8_t  inited;                  /**< 是否已初始化 */
    adhoc_sm_cfg_t cfg;               /**< 配置 */
    uint8_t  role;                    /**< 当前角色 */
    adhoc_sm_state_t state;           /**< 当前状态 */
    uint8_t  joined_level;            /**< 入网级别 */
    uint32_t upstream_id;             /**< 上级ID */
    uint8_t  upstream_no;             /**< 上级绑定编号 */
    uint8_t  upstream_gateway_no;     /**< 上级网关编号 */
    uint8_t  retry_count;             /**< 重试计数 */
    uint32_t next_tx_us;              /**< 下次发送时刻(μs) */
    uint32_t upstream_last_seen_us;   /**< 最近收到上级时刻 */
    uint8_t  gateway_network_started; /**< 网关组网已启动 */
    uint8_t  gateway_network_locked;  /**< 网关组网已锁定 */
    uint32_t gateway_network_start_us;/**< 网关组网启动时刻 */
    uint32_t gateway_network_end_us;  /**< 网关组网结束时刻 */
    uint8_t  gateway_rx_window_open;  /**< 网关接收窗口开启 */
    uint32_t gateway_rx_window_start_us; /**< 网关接收窗口起始 */
    uint32_t gateway_rx_window_end_us;   /**< 网关接收窗口结束 */
    uint8_t  network_lock_active;     /**< 网络锁定激活 */
    uint8_t  network_lock_closed;     /**< 网络锁定关闭 */
    uint32_t network_lock_end_us;     /**< 网络锁定结束时刻 */
    uint8_t  regroup_timer_active;    /**< 重组网定时器激活 */
    uint32_t regroup_start_us;        /**< 重组网起始时刻 */
    adhoc_sm_candidate_t candidate;   /**< 当前候选上级 */
    adhoc_sm_candidate_t neighbor_cache[ADHOC_SM_NEIGHBOR_CACHE_CAPACITY]; /**< 邻居缓存 */
    adhoc_sm_upstream_binding_t upstream_bindings[ADHOC_SM_UPSTREAM_BINDING_CAPACITY]; /**< 上级绑定表 */
    adhoc_sm_downstream_binding_t downstream_bindings[ADHOC_SM_DOWNSTREAM_BINDING_CAPACITY]; /**< 直接下级绑定记录 */
    adhoc_reply_list_t downlink_confirm_list; /**< 下行确认队列 */
} adhoc_sm_t;

/* ========== 状态机 API ========== */

/**
 * @brief 初始化状态机
 * @param sm  状态机实例
 * @param cfg 配置参数
 * @return 1成功 0失败
 */
int adhoc_sm_init(adhoc_sm_t *sm, const adhoc_sm_cfg_t *cfg);

/**
 * @brief 设置角色
 * @param sm   状态机实例
 * @param role 角色
 * @return 1成功 0失败
 */
int adhoc_sm_set_role(adhoc_sm_t *sm, uint8_t role);

/**
 * @brief 复位状态机到 ST1
 * @param sm 状态机实例
 */
void adhoc_sm_reset(adhoc_sm_t *sm);

/**
 * @brief 向状态机注入接收帧事件
 * @param sm    状态机实例
 * @param event 接收事件
 * @return 处理结果(忽略/接受/状态变化)
 */
adhoc_sm_rx_result_t adhoc_sm_on_rx(adhoc_sm_t *sm, const adhoc_sm_rx_event_t *event);

/**
 * @brief 周期轮询: 推进状态机时序与发送调度
 * @param sm         状态机实例
 * @param now_us     当前微秒时间
 * @param out_fields 输出待发帧字段(若需要发送)
 * @param out_has_tx 输出是否有待发帧
 * @return 1正常 0异常
 */
int adhoc_sm_poll(adhoc_sm_t *sm, uint32_t now_us, adhoc_frame_fields_t *out_fields, uint8_t *out_has_tx);

/**
 * @brief 获取状态机快照
 * @param sm          状态机实例
 * @param out_snapshot 输出快照
 */
void adhoc_sm_get_snapshot(const adhoc_sm_t *sm, adhoc_sm_snapshot_t *out_snapshot);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_SM_H */
