/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_node.c
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            协议节点主控模块
**                          对外的唯一入口, 调度状态机(adhoc_sm)与数据面(adhoc_data_plane),
**                          管理 pending_tx 单帧发送模型, 同步路由上下文
**--------------------------------------------------------------------------------------------------------
*/

#include "adhoc_api.h"
#include "adhoc_data_plane.h"
#include "adhoc_frame.h"
#include "adhoc_sm.h"
#include "adhoc_timing.h"

#include <string.h>

/**
 * @brief 节点内部上下文(完全封装, 外部不可见)
 */
typedef struct
{
    uint8_t  inited;             /**< 是否已初始化 */
    adhoc_role_t role;           /**< 当前角色 */
    adhoc_cfg_t cfg;             /**< 节点配置 */
    const adhoc_link_ops_t *link_ops; /**< 链路操作接口 */
    void    *link_ctx;           /**< 链路上下文 */
    adhoc_frame_t pending_tx;    /**< 单帧待发缓冲 */
    uint8_t  has_pending_tx;     /**< 是否有待发帧 */
    adhoc_frame_fields_t last_rx_fields; /**< 最近接收帧解析结果 */
    uint8_t  has_last_rx;        /**< 是否有最近接收帧 */
    adhoc_timing_plan_t timing_plan; /**< 时序方案 */
    adhoc_sm_t sm;               /**< 组网状态机 */
    adhoc_data_plane_t data_plane; /**< 数据面 */
} adhoc_node_ctx_t;

/**
 * @brief 类型安全的节点上下文转换
 */
static adhoc_node_ctx_t *adhoc_cast(void *node)
{
    return (adhoc_node_ctx_t *)node;
}

/**
 * @brief 将状态机的路由信息同步到数据面
 *
 * 对于网关: joined_level=0, upstream_gateway_no=0, upstream_no=0
 * 对于信标: 从 sm 快照中提取路由信息
 */
static int adhoc_node_sync_route_context(adhoc_node_ctx_t *ctx)
{
    adhoc_sm_snapshot_t snapshot;
    adhoc_data_downstream_binding_t downstream_bindings[ADHOC_DATA_DOWNSTREAM_BINDING_CAPACITY];
    uint8_t index;

    if (ctx == NULL || ctx->inited == 0u)
    {
        return 0;
    }
    if (ctx->role == ADHOC_ROLE_GATEWAY)
    {
        return adhoc_data_plane_set_route(&ctx->data_plane, 0u, 0u, 0u, 0u, NULL, 0u);
    }

    memset(&snapshot, 0, sizeof(snapshot));
    memset(downstream_bindings, 0, sizeof(downstream_bindings));
    adhoc_sm_get_snapshot(&ctx->sm, &snapshot);
    for (index = 0u; index < ADHOC_DATA_DOWNSTREAM_BINDING_CAPACITY && index < snapshot.downstream_binding_count; ++index)
    {
        downstream_bindings[index].used = snapshot.downstream_bindings[index].used;
        downstream_bindings[index].child_bind_no = snapshot.downstream_bindings[index].child_bind_no;
        downstream_bindings[index].child_level = snapshot.downstream_bindings[index].child_level;
        downstream_bindings[index].gateway_no = snapshot.downstream_bindings[index].gateway_no;
        downstream_bindings[index].child_id = snapshot.downstream_bindings[index].child_id;
        downstream_bindings[index].last_seen_us = snapshot.downstream_bindings[index].last_seen_us;
    }
    return adhoc_data_plane_set_route(&ctx->data_plane,
                                      snapshot.joined_level,
                                      snapshot.upstream_gateway_no,
                                      snapshot.upstream_no,
                                      snapshot.upstream_id,
                                      downstream_bindings,
                                      snapshot.downstream_binding_count);
}

/**
 * @brief 校验配置参数合法性
 */
static int adhoc_cfg_valid(const adhoc_cfg_t *cfg)
{
    if (cfg == NULL)
    {
        return 0;
    }
    if (cfg->domain_id > 0x07FFu || cfg->node_id == 0u || cfg->node_id > 0x1FFFFFFFu || cfg->gateway_no > 7u)
    {
        return 0;
    }
    if (cfg->retry_max == 0u)
    {
        return 0;
    }
    return 1;
}

/**
 * @brief 获取节点上下文所需最小静态内存大小
 */
uint32_t adhoc_node_required_size(void)
{
    return (uint32_t)sizeof(adhoc_node_ctx_t);
}

/**
 * @brief 初始化协议节点
 *
 * 流程: 校验参数 → 初始化时序方案 → 初始化状态机 → 初始化数据面 → 调用链路 init → 同步路由
 */
adhoc_rc_t adhoc_node_init(void *node_mem, uint32_t node_mem_size,
                           const adhoc_cfg_t *cfg,
                           const adhoc_link_ops_t *link_ops, void *link_ctx)
{
    adhoc_node_ctx_t *node;
    adhoc_timing_cfg_t timing_cfg;
    adhoc_sm_cfg_t sm_cfg;
    adhoc_data_plane_cfg_t data_cfg;
    int init_ret;

    if (node_mem == NULL || link_ops == NULL)
    {
        return ADHOC_EINVAL;
    }
    if (node_mem_size < (uint32_t)sizeof(adhoc_node_ctx_t) || !adhoc_cfg_valid(cfg))
    {
        return ADHOC_EINVAL;
    }

    node = adhoc_cast(node_mem);
    memset(node, 0, sizeof(*node));
    node->cfg = *cfg;
    node->role = ADHOC_ROLE_BEACON;
    node->link_ops = link_ops;
    node->link_ctx = link_ctx;

    /* 构建时序方案 */
    timing_cfg.t1_us = cfg->t1_us;
    timing_cfg.t2_us = cfg->t2_us;
    timing_cfg.t3_us = cfg->t3_us;
    timing_cfg.t4_us = cfg->t4_us;
    if (!adhoc_timing_build(&timing_cfg, &node->timing_plan))
    {
        return ADHOC_EINVAL;
    }

    /* 初始化状态机 */
    sm_cfg.domain_id          = cfg->domain_id;
    sm_cfg.node_id            = cfg->node_id;
    sm_cfg.gateway_no         = cfg->gateway_no;
    sm_cfg.t2_us              = node->timing_plan.t2_us;
    sm_cfg.t5_us              = node->timing_plan.t5_us;
    sm_cfg.retry_max          = cfg->retry_max;
    sm_cfg.network_window_us  = cfg->network_window_us;
    sm_cfg.regroup_interval_us = cfg->regroup_interval_us;
    sm_cfg.role               = ADHOC_SM_ROLE_BEACON;
    if (!adhoc_sm_init(&node->sm, &sm_cfg))
    {
        return ADHOC_EINVAL;
    }

    /* 初始化数据面 */
    data_cfg.domain_id     = cfg->domain_id;
    data_cfg.node_id       = cfg->node_id;
    data_cfg.gateway_no    = cfg->gateway_no;
    data_cfg.role_gateway  = 0u;
    data_cfg.t5_us         = node->timing_plan.t5_us;
    data_cfg.dedup_window_ms = 0u;
    data_cfg.tx_retry_max  = cfg->retry_max;
    if (!adhoc_data_plane_init(&node->data_plane, &data_cfg))
    {
        return ADHOC_EINVAL;
    }

    /* 调用链路层初始化 */
    if (node->link_ops->init != NULL)
    {
        init_ret = node->link_ops->init(node->link_ctx);
        if (init_ret != 0)
        {
            return ADHOC_ESTATE;
        }
    }

    node->inited = 1u;
    if (!adhoc_node_sync_route_context(node))
    {
        return ADHOC_ESTATE;
    }
    return ADHOC_OK;
}

/**
 * @brief 设置节点角色(网关/信标), 同时切换状态机和数据面角色
 */
adhoc_rc_t adhoc_node_set_role(void *node, adhoc_role_t role)
{
    adhoc_node_ctx_t *ctx = adhoc_cast(node);

    if (ctx == NULL || ctx->inited == 0u)
    {
        return ADHOC_ESTATE;
    }
    if (role != ADHOC_ROLE_BEACON && role != ADHOC_ROLE_GATEWAY)
    {
        return ADHOC_EINVAL;
    }

    ctx->role = role;
    if (!adhoc_sm_set_role(&ctx->sm, role == ADHOC_ROLE_GATEWAY ? ADHOC_SM_ROLE_GATEWAY : ADHOC_SM_ROLE_BEACON))
    {
        return ADHOC_ESTATE;
    }
    if (!adhoc_data_plane_set_role(&ctx->data_plane, role == ADHOC_ROLE_GATEWAY ? 1u : 0u))
    {
        return ADHOC_ESTATE;
    }
    if (!adhoc_node_sync_route_context(ctx))
    {
        return ADHOC_ESTATE;
    }
    return ADHOC_OK;
}

/**
 * @brief 复位节点状态(回到 ST1)
 */
adhoc_rc_t adhoc_node_reset(void *node)
{
    adhoc_node_ctx_t *ctx = adhoc_cast(node);

    if (ctx == NULL || ctx->inited == 0u)
    {
        return ADHOC_ESTATE;
    }

    ctx->has_pending_tx = 0u;
    memset(&ctx->pending_tx, 0, sizeof(ctx->pending_tx));
    ctx->has_last_rx = 0u;
    memset(&ctx->last_rx_fields, 0, sizeof(ctx->last_rx_fields));
    adhoc_sm_reset(&ctx->sm);
    adhoc_data_plane_reset(&ctx->data_plane);
    if (!adhoc_node_sync_route_context(ctx))
    {
        return ADHOC_ESTATE;
    }
    return ADHOC_OK;
}

/**
 * @brief 向协议层注入接收帧
 *
 * 校验流程: 长度 → CRC8 → 域号 → 时隙奇偶 → A帧入SM / D帧入数据面
 */
adhoc_rc_t adhoc_node_on_rx(void *node, const adhoc_frame_t *rx)
{
    adhoc_node_ctx_t *ctx = adhoc_cast(node);
    adhoc_frame_fields_t parsed;
    adhoc_data_rx_result_t data_rc;
    adhoc_sm_rx_event_t event;

    if (ctx == NULL || rx == NULL)
    {
        return ADHOC_EINVAL;
    }
    if (ctx->inited == 0u)
    {
        return ADHOC_ESTATE;
    }
    if (rx->len != ADHOC_FRAME_LEN)
    {
        return ADHOC_EINVAL;
    }
    if (!adhoc_frame_parse(rx->bytes, &parsed))
    {
        return ADHOC_EINVAL;
    }
    if (parsed.sender.domain_id != ctx->cfg.domain_id)
    {
        return ADHOC_EINVAL;
    }
    if (!adhoc_timing_slot_parity_match(parsed.level, parsed.slot_high4))
    {
        return ADHOC_EINVAL;
    }

    ctx->last_rx_fields = parsed;
    ctx->has_last_rx = 1u;

    /* D 帧 → 数据面处理 */
    if (parsed.msg_class == ADHOC_MSG_CLASS_D)
    {
        if (!adhoc_node_sync_route_context(ctx))
        {
            return ADHOC_ESTATE;
        }
        data_rc = adhoc_data_plane_on_rx(&ctx->data_plane, &parsed, rx->ts_us);
        if (data_rc == ADHOC_DATA_RX_IGNORED)
        {
            return ADHOC_EINVAL;
        }
        return ADHOC_OK;
    }

    /* A 帧 → 状态机处理 */
    if (parsed.msg_class == ADHOC_MSG_CLASS_A &&
        (ctx->role == ADHOC_ROLE_BEACON || ctx->role == ADHOC_ROLE_GATEWAY))
    {
        memset(&event, 0, sizeof(event));
        event.msg_class  = parsed.msg_class;
        event.gateway_no = parsed.gateway_no;
        event.level      = parsed.level;
        event.sender     = parsed.sender;
        memcpy(event.content, parsed.content, sizeof(event.content));
        event.rssi  = rx->rssi;
        event.ts_us = rx->ts_us;
        (void)adhoc_sm_on_rx(&ctx->sm, &event);
        if (!adhoc_node_sync_route_context(ctx))
        {
            return ADHOC_ESTATE;
        }
    }

    return ADHOC_OK;
}

/**
 * @brief 周期驱动: 启动接收 → 网关ACK调度 → SM调度 → 数据转发调度
 *
 * 发送优先级: 网关ACK > 组网帧(A) > 数据转发(D)
 * 采用单 pending_tx 模式, 同一周期只保留一帧待发
 */
adhoc_rc_t adhoc_node_poll(void *node, uint32_t now_us)
{
    adhoc_node_ctx_t *ctx = adhoc_cast(node);
    adhoc_frame_fields_t data_tx_fields;
    uint8_t data_has_tx = 0u;
    adhoc_frame_fields_t tx_fields;
    uint8_t has_tx = 0u;
    adhoc_frame_fields_t forward_tx_fields;
    uint8_t forward_has_tx = 0u;

    if (ctx == NULL || ctx->inited == 0u)
    {
        return ADHOC_ESTATE;
    }

    /* 启动链路接收 */
    if (ctx->link_ops != NULL && ctx->link_ops->start_rx != NULL)
    {
        if (ctx->link_ops->start_rx(ctx->link_ctx) != ADHOC_LINK_OK)
        {
            return ADHOC_ESTATE;
        }
    }

    /* 优先级1: 网关 ACK */
    if (ctx->role == ADHOC_ROLE_GATEWAY)
    {
        if (!adhoc_node_sync_route_context(ctx))
        {
            return ADHOC_ESTATE;
        }
        if (!adhoc_data_plane_poll_gateway_ack(&ctx->data_plane, now_us, &data_tx_fields, &data_has_tx))
        {
            return ADHOC_ESTATE;
        }
        if (data_has_tx != 0u)
        {
            if (ctx->has_pending_tx != 0u)
            {
                return ADHOC_EBUSY;
            }
            memset(&ctx->pending_tx, 0, sizeof(ctx->pending_tx));
            ctx->pending_tx.len = ADHOC_FRAME_LEN;
            ctx->pending_tx.ts_us = now_us;
            if (!adhoc_frame_build(&data_tx_fields, ctx->pending_tx.bytes))
            {
                return ADHOC_ESTATE;
            }
            ctx->has_pending_tx = 1u;
            return ADHOC_OK;
        }
    }

    /* 优先级2: 组网帧 (A) */
    if (ctx->role == ADHOC_ROLE_BEACON || ctx->role == ADHOC_ROLE_GATEWAY)
    {
        if (!adhoc_sm_poll(&ctx->sm, now_us, &tx_fields, &has_tx))
        {
            return ADHOC_ESTATE;
        }
        if (!adhoc_node_sync_route_context(ctx))
        {
            return ADHOC_ESTATE;
        }
        if (has_tx != 0u)
        {
            if (ctx->has_pending_tx != 0u)
            {
                return ADHOC_EBUSY;
            }
            memset(&ctx->pending_tx, 0, sizeof(ctx->pending_tx));
            ctx->pending_tx.len = ADHOC_FRAME_LEN;
            ctx->pending_tx.ts_us = now_us;
            if (!adhoc_frame_build(&tx_fields, ctx->pending_tx.bytes))
            {
                return ADHOC_ESTATE;
            }
            ctx->has_pending_tx = 1u;
        }
    }

    /* 优先级3: 数据转发 (D) */
    if (ctx->has_pending_tx == 0u)
    {
        if (!adhoc_node_sync_route_context(ctx))
        {
            return ADHOC_ESTATE;
        }
        if (!adhoc_data_plane_poll_forward_tx(&ctx->data_plane, now_us, &forward_tx_fields, &forward_has_tx))
        {
            return ADHOC_ESTATE;
        }
        if (forward_has_tx != 0u)
        {
            memset(&ctx->pending_tx, 0, sizeof(ctx->pending_tx));
            ctx->pending_tx.len = ADHOC_FRAME_LEN;
            ctx->pending_tx.ts_us = now_us;
            if (!adhoc_frame_build(&forward_tx_fields, ctx->pending_tx.bytes))
            {
                return ADHOC_ESTATE;
            }
            ctx->has_pending_tx = 1u;
        }
    }

    return ADHOC_OK;
}

/**
 * @brief 拉取待发帧(由 poll 生成)
 *
 * 取出 pending_tx 后清空, 外部通过 link_ops.tx 将帧发出
 */
adhoc_rc_t adhoc_node_fetch_tx(void *node, adhoc_frame_t *tx)
{
    adhoc_node_ctx_t *ctx = adhoc_cast(node);

    if (ctx == NULL || tx == NULL)
    {
        return ADHOC_EINVAL;
    }
    if (ctx->inited == 0u)
    {
        return ADHOC_ESTATE;
    }
    if (ctx->has_pending_tx == 0u)
    {
        return ADHOC_ENOFRAME;
    }

    *tx = ctx->pending_tx;
    ctx->has_pending_tx = 0u;
    memset(&ctx->pending_tx, 0, sizeof(ctx->pending_tx));
    return ADHOC_OK;
}

/**
 * @brief 提交业务数据(仅已入网信标可用)
 *
 * @note lmt_d=0 时自动从 now_us 生成, 数据进入待发队列后由数据面负责重试与确认
 */
adhoc_rc_t adhoc_node_submit_data(void *node, uint8_t source_id_flag, uint32_t lmt_d,
                                  const uint8_t user[ADHOC_DATA_USER_LEN], uint32_t now_us)
{
    adhoc_node_ctx_t *ctx = adhoc_cast(node);
    adhoc_sm_snapshot_t snapshot;
    uint32_t submit_lmt_d;

    if (ctx == NULL || user == NULL)
    {
        return ADHOC_EINVAL;
    }
    if (ctx->inited == 0u)
    {
        return ADHOC_ESTATE;
    }
    if (source_id_flag > ADHOC_PAYLOAD_ID_FLAG_BIND_MAX)
    {
        return ADHOC_EINVAL;
    }
    if (ctx->role != ADHOC_ROLE_BEACON)
    {
        return ADHOC_ESTATE;
    }

    memset(&snapshot, 0, sizeof(snapshot));
    adhoc_sm_get_snapshot(&ctx->sm, &snapshot);
    if (snapshot.joined_level == 0u || snapshot.upstream_id == 0u || snapshot.upstream_gateway_no > 7u)
    {
        return ADHOC_ESTATE;
    }

    submit_lmt_d = lmt_d == 0u ? adhoc_lmt_d_from_us(now_us) : (lmt_d & ADHOC_DATA_LMT_D_MAX);
    if (!adhoc_data_plane_submit_source_data(&ctx->data_plane, source_id_flag, submit_lmt_d, user,
                                             snapshot.joined_level, snapshot.upstream_gateway_no, now_us))
    {
        return ADHOC_EBUSY;
    }
    return ADHOC_OK;
}

/**
 * @brief 拉取数据发送结果报告
 */
adhoc_rc_t adhoc_node_fetch_data_tx_report(void *node, adhoc_node_data_tx_report_t *out_report)
{
    adhoc_node_ctx_t *ctx = adhoc_cast(node);
    adhoc_data_tx_report_t report;

    if (ctx == NULL || out_report == NULL)
    {
        return ADHOC_EINVAL;
    }
    if (ctx->inited == 0u)
    {
        return ADHOC_ESTATE;
    }
    if (!adhoc_data_plane_pop_tx_report(&ctx->data_plane, &report))
    {
        return ADHOC_ENOFRAME;
    }

    memset(out_report, 0, sizeof(*out_report));
    if (report.code == ADHOC_DATA_TX_REPORT_ACKED)
    {
        out_report->code = ADHOC_NODE_DATA_TX_REPORT_ACKED;
    }
    else if (report.code == ADHOC_DATA_TX_REPORT_RETRY_EXHAUSTED)
    {
        out_report->code = ADHOC_NODE_DATA_TX_REPORT_RETRY_EXHAUSTED;
    }
    else
    {
        out_report->code = ADHOC_NODE_DATA_TX_REPORT_NONE;
    }
    out_report->source_id_flag = report.item.source.id_flag;
    out_report->source_node_id = report.item.source.node_id;
    out_report->lmt_d          = report.item.lmt_d;
    out_report->retry_count    = report.retry_count;
    return ADHOC_OK;
}

/**
 * @brief 获取节点运行态快照(从状态机提取)
 */
adhoc_rc_t adhoc_node_get_runtime_status(void *node, adhoc_node_runtime_status_t *out_status)
{
    adhoc_node_ctx_t *ctx = adhoc_cast(node);
    adhoc_sm_snapshot_t snapshot;

    if (ctx == NULL || out_status == NULL)
    {
        return ADHOC_EINVAL;
    }
    if (ctx->inited == 0u)
    {
        return ADHOC_ESTATE;
    }

    memset(&snapshot, 0, sizeof(snapshot));
    memset(out_status, 0, sizeof(*out_status));
    adhoc_sm_get_snapshot(&ctx->sm, &snapshot);

    out_status->state                    = (uint8_t)snapshot.state;
    out_status->joined_level             = snapshot.joined_level;
    out_status->upstream_id              = snapshot.upstream_id;
    out_status->upstream_no              = snapshot.upstream_no;
    out_status->upstream_gateway_no      = snapshot.upstream_gateway_no;
    out_status->retry_count              = snapshot.retry_count;
    out_status->upstream_last_seen_us    = snapshot.upstream_last_seen_us;
    out_status->gateway_network_started  = snapshot.gateway_network_started;
    out_status->gateway_network_locked   = snapshot.gateway_network_locked;
    out_status->gateway_network_start_us = snapshot.gateway_network_start_us;
    out_status->gateway_network_end_us   = snapshot.gateway_network_end_us;
    out_status->network_lock_active      = snapshot.network_lock_active;
    out_status->network_lock_closed      = snapshot.network_lock_closed;
    out_status->network_lock_end_us      = snapshot.network_lock_end_us;
    return ADHOC_OK;
}
