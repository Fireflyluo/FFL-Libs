/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_api.h
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            Ad-Hoc-lib 协议库对外 API 接口
**                          提供节点初始化、角色设置、帧收发、数据提交、状态查询等全部对外功能
**--------------------------------------------------------------------------------------------------------
*/

#ifndef ADHOC_API_H
#define ADHOC_API_H

#include "adhoc_link.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 协议帧固定长度(字节) */
#define ADHOC_FRAME_LEN 32u
/** @brief 数据帧中用户数据字段长度(字节) */
#define ADHOC_DATA_USER_LEN 17u

/** @brief 节点角色枚举 */
typedef enum
{
    ADHOC_ROLE_BEACON  = 0,  /**< 信标节点 */
    ADHOC_ROLE_GATEWAY = 1   /**< 网关节点 */
} adhoc_role_t;

/** @brief 协议操作返回码 */
typedef enum
{
    ADHOC_OK       =  0,  /**< 操作成功 */
    ADHOC_EINVAL   = -1,  /**< 参数非法 */
    ADHOC_ESTATE   = -2,  /**< 状态非法 / 未初始化 / 角色不匹配 */
    ADHOC_EBUSY    = -3,  /**< 资源忙(如 pending_tx 已占用) */
    ADHOC_ENOFRAME = -4   /**< 当前无待发帧 / 无报告可取 */
} adhoc_rc_t;

/**
 * @brief 节点配置参数结构体
 */
typedef struct
{
    uint16_t domain_id;          /**< 组网域号(0~2047) */
    uint32_t node_id;            /**< 节点 ID(29bit, 非0) */
    uint8_t  gateway_no;         /**< 网关编号(0~7) */
    uint32_t t1_us;              /**< 时隙宽度(μs) */
    uint32_t t2_us;              /**< 接收窗口时长(μs) */
    uint32_t t3_us;              /**< 探测侦听时间(μs) */
    uint32_t t4_us;              /**< 探测休眠时间(μs) */
    uint8_t  retry_max;          /**< 入网/数据重试上限 */
    uint32_t network_window_us;  /**< 组网时间窗(μs), 建议30s起步 */
    uint32_t regroup_interval_us;/**< 重组网周期(μs), 0=关闭 */
} adhoc_cfg_t;

/**
 * @brief 收发帧描述结构体
 */
typedef struct
{
    uint8_t  bytes[ADHOC_FRAME_LEN]; /**< 帧原始字节(32B) */
    uint8_t  len;                    /**< 帧长度(固定32) */
    int8_t   rssi;                   /**< 接收信号强度(dBm) */
    uint32_t ts_us;                  /**< 接收/发送时刻微秒时间戳 */
} adhoc_frame_t;

/** @brief 数据发送结果报告码 */
typedef enum
{
    ADHOC_NODE_DATA_TX_REPORT_NONE            = 0u, /**< 无报告 */
    ADHOC_NODE_DATA_TX_REPORT_ACKED           = 1u, /**< 数据已被确认 */
    ADHOC_NODE_DATA_TX_REPORT_RETRY_EXHAUSTED = 2u  /**< 重试耗尽 */
} adhoc_node_data_tx_report_code_t;

/**
 * @brief 数据发送结果报告
 */
typedef struct
{
    uint8_t  code;            /**< 报告码, 见 adhoc_node_data_tx_report_code_t */
    uint8_t  source_id_flag;  /**< 源数据 ID 标志 */
    uint32_t source_node_id;  /**< 源节点 ID */
    uint32_t lmt_d;           /**< 数据时间戳(灵码标 LMT_D) */
    uint8_t  retry_count;     /**< 实际重试次数 */
} adhoc_node_data_tx_report_t;

/**
 * @brief 节点运行态快照
 */
typedef struct
{
    uint8_t  state;                    /**< 当前状态机状态 */
    uint8_t  joined_level;             /**< 已入网级别(0=未入网) */
    uint32_t upstream_id;              /**< 上级节点 ID */
    uint8_t  upstream_no;              /**< 上级绑定编号(0~5) */
    uint8_t  upstream_gateway_no;      /**< 上级所属网关编号 */
    uint8_t  retry_count;              /**< 当前重试计数 */
    uint32_t upstream_last_seen_us;    /**< 最近一次收到上级帧的微秒时间戳 */
    uint8_t  gateway_network_started;  /**< 网关: 组网是否已启动 */
    uint8_t  gateway_network_locked;   /**< 网关: 组网是否已锁定 */
    uint32_t gateway_network_start_us; /**< 网关: 组网启动时刻(μs) */
    uint32_t gateway_network_end_us;   /**< 网关: 组网结束时刻(μs) */
    uint8_t  network_lock_active;      /**< 信标: 网络锁定是否激活 */
    uint8_t  network_lock_closed;      /**< 信标: 网络锁定是否关闭 */
    uint32_t network_lock_end_us;      /**< 信标: 网络锁定结束时刻(μs) */
} adhoc_node_runtime_status_t;

/* ========== API 函数声明 ========== */

/**
 * @brief 获取节点上下文所需的最小静态内存大小
 * @return 最小字节数
 */
uint32_t adhoc_node_required_size(void);

/**
 * @brief 初始化协议节点
 * @param node_mem     外部提供的静态内存缓冲区
 * @param node_mem_size 缓冲区大小(应 >= adhoc_node_required_size())
 * @param cfg          节点配置参数
 * @param link_ops     链路层操作接口
 * @param link_ctx     链路层上下文(将透传给 link_ops 各函数)
 * @return ADHOC_OK 成功, 其他值表示失败
 */
adhoc_rc_t adhoc_node_init(void *node_mem, uint32_t node_mem_size,
                           const adhoc_cfg_t *cfg,
                           const adhoc_link_ops_t *link_ops, void *link_ctx);

/**
 * @brief 设置节点角色(网关/信标)
 * @param node 已初始化的节点实例
 * @param role 目标角色
 * @return ADHOC_OK 成功
 */
adhoc_rc_t adhoc_node_set_role(void *node, adhoc_role_t role);

/**
 * @brief 复位节点状态(回到 ST1)
 * @param node 节点实例
 * @return ADHOC_OK 成功
 */
adhoc_rc_t adhoc_node_reset(void *node);

/**
 * @brief 向协议层注入接收帧
 * @param node 节点实例
 * @param rx   接收帧描述(含32B原始数据、RSSI、时间戳)
 * @return ADHOC_OK 成功, ADHOC_EINVAL 帧校验失败
 */
adhoc_rc_t adhoc_node_on_rx(void *node, const adhoc_frame_t *rx);

/**
 * @brief 周期驱动: 推进状态机并调度发送
 * @param node   节点实例
 * @param now_us 当前微秒时间戳
 * @return ADHOC_OK 或 ADHOC_EBUSY
 */
adhoc_rc_t adhoc_node_poll(void *node, uint32_t now_us);

/**
 * @brief 拉取待发帧(由 poll 生成)
 * @param node 节点实例
 * @param tx   输出待发帧
 * @return ADHOC_OK 成功, ADHOC_ENOFRAME 无待发帧
 */
adhoc_rc_t adhoc_node_fetch_tx(void *node, adhoc_frame_t *tx);

/**
 * @brief 提交业务数据(仅信标角色可用)
 * @param node           节点实例
 * @param source_id_flag 源数据 ID 标志(0~7)
 * @param lmt_d          数据时间戳(0=自动生成)
 * @param user           用户数据(17B)
 * @param now_us         当前微秒时间戳
 * @return ADHOC_OK 成功, ADHOC_ESTATE 未入网不允许提交
 */
adhoc_rc_t adhoc_node_submit_data(void *node, uint8_t source_id_flag, uint32_t lmt_d,
                                  const uint8_t user[ADHOC_DATA_USER_LEN], uint32_t now_us);

/**
 * @brief 拉取数据发送结果报告
 * @param node       节点实例
 * @param out_report 输出报告
 * @return ADHOC_OK 成功, ADHOC_ENOFRAME 无报告
 */
adhoc_rc_t adhoc_node_fetch_data_tx_report(void *node, adhoc_node_data_tx_report_t *out_report);

/**
 * @brief 获取节点运行态快照
 * @param node       节点实例
 * @param out_status 输出运行态信息
 * @return ADHOC_OK 成功
 */
adhoc_rc_t adhoc_node_get_runtime_status(void *node, adhoc_node_runtime_status_t *out_status);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_API_H */
