/*-----------------------------------------------File Info----------------------------------------
** File Name:               adhoc_data_plane.c
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            数据面实现
**                          管理去重表、ACK队列、数据待发队列、TX报告队列,
**                          实现源数据提交、重试、监听确认、网关ACK汇聚、数据转发
**-------------------------------------------------------------------------------------------------
*/

#include "adhoc_data_plane.h"

#include <string.h>

/**
 * @brief 微秒转毫秒
 */
static uint32_t adhoc_us_to_ms(uint32_t ts_us)
{
    return ts_us / 1000u;
}

static int adhoc_data_route_valid(const adhoc_data_plane_t *plane);

/** @brief D 帧可转发的绑定编号范围检查(flag 仅允许 0~5) */
static int adhoc_data_bind_flag_forwardable(uint8_t id_flag)
{
    return id_flag <= ADHOC_PAYLOAD_ID_FLAG_BIND_MAX ? 1 : 0;
}

/**
 * @brief 校验ACK条目合法性
 */
static int adhoc_data_ack_item_valid(const adhoc_data_ack_item_t *item)
{
    if (item == 0) return 0;
    if (item->source.id_flag > ADHOC_PAYLOAD_ID_FLAG_MAX) return 0;
    if (item->source.node_id == 0u || item->source.node_id > ADHOC_SENDER_NODE_MAX) return 0;
    if (item->lmt_d > ADHOC_DATA_LMT_D_MAX) return 0;
    return 1;
}

/**
 * @brief 比较两个数据源键值(source.node_id + lmt_d)
 */
static int adhoc_data_source_key_equal(const adhoc_payload_id_t *left_source, uint32_t left_lmt_d,
                                       const adhoc_payload_id_t *right_source, uint32_t right_lmt_d)
{
    if (left_source == 0 || right_source == 0) return 0;
    return left_source->node_id == right_source->node_id && left_lmt_d == right_lmt_d ? 1 : 0;
}

/** @brief 获取本地上级绑定编号 */
static uint8_t adhoc_data_local_upstream_no(const adhoc_data_plane_t *plane)
{
    return plane == 0 ? 0u : plane->upstream_no;
}

/** @brief 查找直接下级绑定记录(按发送者视角关系匹配) */
static const adhoc_data_downstream_binding_t *adhoc_data_find_downstream_binding(const adhoc_data_plane_t *plane,
                                                                                 const adhoc_frame_fields_t *fields)
{
    uint8_t index;
    const adhoc_data_downstream_binding_t *binding;
    if (plane == 0 || fields == 0) return 0;
    for (index = 0u; index < ADHOC_DATA_DOWNSTREAM_BINDING_CAPACITY; ++index) {
        binding = &plane->downstream_bindings[index];
        if (binding->used == 0u) continue;
        if (binding->child_id != fields->sender.node_id) continue;
        if (binding->child_level != fields->level) continue;
        if (binding->gateway_no != fields->gateway_no) continue;
        return binding;
    }
    return 0;
}

/** @brief 判断发送者是否为当前直接上级 */
static int adhoc_data_sender_is_upstream(const adhoc_data_plane_t *plane, const adhoc_frame_fields_t *fields)
{
    if (plane == 0 || fields == 0) return 0;
    if (!adhoc_data_route_valid(plane)) return 0;
    if (plane->upstream_id == 0u) return 0;
    if (fields->sender.node_id != plane->upstream_id) return 0;
    if (fields->gateway_no != plane->upstream_gateway_no) return 0;
    return fields->level + 1u == plane->joined_level ? 1 : 0;
}

/** @brief 检查信标路由是否有效(已入网且有上级) */
static int adhoc_data_route_valid(const adhoc_data_plane_t *plane)
{
    if (plane == 0) return 0;
    if (plane->role_gateway != 0u) return 0;
    if (plane->joined_level == 0u || plane->upstream_gateway_no > 7u) return 0;
    return 1;
}

/** @brief 校验数据消息合法性 */
static int adhoc_data_msg_valid(const adhoc_data_msg_t *msg)
{
    if (msg == 0) return 0;
    if (msg->source.id_flag > ADHOC_PAYLOAD_ID_FLAG_MAX) return 0;
    if (msg->source.node_id == 0u || msg->source.node_id > ADHOC_SENDER_NODE_MAX) return 0;
    if (msg->lmt_d > ADHOC_DATA_LMT_D_MAX) return 0;
    return 1;
}

/* ========== ACK 队列操作 ========== */

/** @brief 比较两个ACK条目是否相等 */
static int adhoc_data_ack_item_equal(const adhoc_data_ack_item_t *left, const adhoc_data_ack_item_t *right)
{
    if (left == 0 || right == 0) return 0;
    return adhoc_data_source_key_equal(&left->source, left->lmt_d, &right->source, right->lmt_d);
}

/** @brief 检查ACK队列是否已包含指定条目(去重) */
static int adhoc_data_ack_queue_contains(const adhoc_data_plane_t *plane, const adhoc_data_ack_item_t *item)
{
    uint8_t index, pos;
    if (plane == 0 || item == 0) return 0;
    for (index = 0u; index < plane->ack_count; ++index) {
        pos = (uint8_t)((plane->ack_head + index) % ADHOC_DATA_ACK_QUEUE_CAPACITY);
        if (!plane->ack_queue[pos].used) continue;
        if (adhoc_data_ack_item_equal(&plane->ack_queue[pos].item, item)) return 1;
    }
    return 0;
}

/** @brief 向ACK队列推入唯一条目 */
static int adhoc_data_ack_queue_push_unique(adhoc_data_plane_t *plane, const adhoc_data_ack_item_t *item)
{
    if (plane == 0 || item == 0) return 0;
    if (!adhoc_data_ack_item_valid(item)) return 0;
    if (adhoc_data_ack_queue_contains(plane, item)) return 1;
    if (plane->ack_count >= ADHOC_DATA_ACK_QUEUE_CAPACITY) return 0;
    plane->ack_queue[plane->ack_tail].used = 1u;
    plane->ack_queue[plane->ack_tail].item = *item;
    plane->ack_tail = (uint8_t)((plane->ack_tail + 1u) % ADHOC_DATA_ACK_QUEUE_CAPACITY);
    plane->ack_count++;
    return 1;
}

/** @brief 从ACK队列弹出一个条目 */
static int adhoc_data_ack_queue_pop(adhoc_data_plane_t *plane, adhoc_data_ack_item_t *item_out)
{
    if (plane == 0 || item_out == 0 || plane->ack_count == 0u) return 0;
    if (!plane->ack_queue[plane->ack_head].used) return 0;
    *item_out = plane->ack_queue[plane->ack_head].item;
    memset(&plane->ack_queue[plane->ack_head], 0, sizeof(plane->ack_queue[plane->ack_head]));
    plane->ack_head = (uint8_t)((plane->ack_head + 1u) % ADHOC_DATA_ACK_QUEUE_CAPACITY);
    plane->ack_count--;
    return 1;
}

/* ========== TX 队列操作 ========== */

/** @brief 比较两个数据消息是否相等(按source+lmt_d) */
static int adhoc_data_tx_msg_equal(const adhoc_data_msg_t *left, const adhoc_data_msg_t *right)
{
    if (left == 0 || right == 0) return 0;
    return adhoc_data_source_key_equal(&left->source, left->lmt_d, &right->source, right->lmt_d);
}

/** @brief 检查待发条目是否到发送时间 */
static int adhoc_data_tx_entry_due(const adhoc_data_tx_entry_t *entry, uint32_t now_us)
{
    if (entry == 0 || entry->used == 0u) return 0;
    return now_us >= entry->next_tx_us ? 1 : 0;
}

/* ========== TX 报告队列操作 ========== */

/** @brief 推入TX报告(满时丢弃最旧的) */
static void adhoc_data_tx_report_push(adhoc_data_plane_t *plane, uint8_t code,
                                      const adhoc_data_ack_item_t *item, uint8_t retry_count)
{
    adhoc_data_tx_report_entry_t *entry;
    if (plane == 0 || item == 0 || code == ADHOC_DATA_TX_REPORT_NONE) return;
    if (!adhoc_data_ack_item_valid(item)) return;
    if (plane->tx_report_count >= ADHOC_DATA_TX_REPORT_QUEUE_CAPACITY) {
        plane->tx_report_head = (uint8_t)((plane->tx_report_head + 1u) % ADHOC_DATA_TX_REPORT_QUEUE_CAPACITY);
        plane->tx_report_count--;
    }
    entry = &plane->tx_report_queue[plane->tx_report_tail];
    memset(entry, 0, sizeof(*entry));
    entry->used = 1u;
    entry->report.code        = code;
    entry->report.item        = *item;
    entry->report.retry_count = retry_count;
    plane->tx_report_tail = (uint8_t)((plane->tx_report_tail + 1u) % ADHOC_DATA_TX_REPORT_QUEUE_CAPACITY);
    plane->tx_report_count++;
}

/** @brief 弹出TX报告 */
static int adhoc_data_tx_report_pop(adhoc_data_plane_t *plane, adhoc_data_tx_report_t *out_report)
{
    adhoc_data_tx_report_entry_t *entry;
    if (plane == 0 || out_report == 0 || plane->tx_report_count == 0u) return 0;
    entry = &plane->tx_report_queue[plane->tx_report_head];
    if (!entry->used) return 0;
    *out_report = entry->report;
    memset(entry, 0, sizeof(*entry));
    plane->tx_report_head = (uint8_t)((plane->tx_report_head + 1u) % ADHOC_DATA_TX_REPORT_QUEUE_CAPACITY);
    plane->tx_report_count--;
    return 1;
}

/** @brief 向待发队列推入唯一数据消息(按source+lmt_d去重) */
static int adhoc_data_tx_queue_push_unique(adhoc_data_plane_t *plane, const adhoc_data_msg_t *msg,
                                           uint8_t level, uint8_t gateway_no, uint32_t now_us)
{
    uint8_t index;
    adhoc_data_tx_entry_t *entry;
    if (plane == 0 || msg == 0) return 0;
    if (!adhoc_data_msg_valid(msg) || level == 0u || gateway_no > 7u) return 0;
    for (index = 0u; index < ADHOC_DATA_TX_QUEUE_CAPACITY; ++index) {
        entry = &plane->tx_queue[index];
        if (!entry->used) continue;
        if (adhoc_data_tx_msg_equal(&entry->msg, msg)) return 1;
    }
    for (index = 0u; index < ADHOC_DATA_TX_QUEUE_CAPACITY; ++index) {
        entry = &plane->tx_queue[index];
        if (entry->used) continue;
        memset(entry, 0, sizeof(*entry));
        entry->used = 1u;
        entry->msg  = *msg;
        entry->level      = level;
        entry->gateway_no = gateway_no;
        entry->retry_count = 0u;
        entry->next_tx_us = now_us;
        return 1;
    }
    return 0;
}

/** @brief 按ACK条目移除待发队列中匹配的条目并推送ACKED报告 */
static uint8_t adhoc_data_tx_queue_remove_by_ack(adhoc_data_plane_t *plane, const adhoc_data_ack_item_t *item)
{
    uint8_t index, removed = 0u;
    adhoc_data_tx_entry_t *entry;
    adhoc_data_ack_item_t report_item;
    if (plane == 0 || item == 0 || !adhoc_data_ack_item_valid(item)) return 0u;
    for (index = 0u; index < ADHOC_DATA_TX_QUEUE_CAPACITY; ++index) {
        entry = &plane->tx_queue[index];
        if (!entry->used) continue;
        if (adhoc_data_source_key_equal(&entry->msg.source, entry->msg.lmt_d, &item->source, item->lmt_d)) {
            report_item.source = entry->msg.source;
            report_item.lmt_d  = entry->msg.lmt_d;
            adhoc_data_tx_report_push(plane, ADHOC_DATA_TX_REPORT_ACKED, &report_item, entry->retry_count);
            memset(entry, 0, sizeof(*entry));
            removed++;
        }
    }
    return removed;
}

/** @brief 按监听到的转发数据移除待发队列中匹配的条目(监听确认) */
static uint8_t adhoc_data_tx_queue_remove_by_forward(adhoc_data_plane_t *plane, const adhoc_data_msg_t *msg)
{
    uint8_t index, removed = 0u;
    adhoc_data_tx_entry_t *entry;
    adhoc_data_ack_item_t report_item;
    if (plane == 0 || msg == 0 || !adhoc_data_msg_valid(msg)) return 0u;
    for (index = 0u; index < ADHOC_DATA_TX_QUEUE_CAPACITY; ++index) {
        entry = &plane->tx_queue[index];
        if (!entry->used) continue;
        if (adhoc_data_source_key_equal(&entry->msg.source, entry->msg.lmt_d, &msg->source, msg->lmt_d)) {
            report_item.source = entry->msg.source;
            report_item.lmt_d  = entry->msg.lmt_d;
            adhoc_data_tx_report_push(plane, ADHOC_DATA_TX_REPORT_ACKED, &report_item, entry->retry_count);
            memset(entry, 0, sizeof(*entry));
            removed++;
        }
    }
    return removed;
}

/** @brief 选取最早到期且重试次数最少的待发条目 */
static int adhoc_data_tx_queue_pick_due(const adhoc_data_plane_t *plane, uint32_t now_us, uint8_t *index_out)
{
    uint8_t index, picked = 0u, has_picked = 0u;
    const adhoc_data_tx_entry_t *entry;
    if (plane == 0 || index_out == 0) return 0;
    for (index = 0u; index < ADHOC_DATA_TX_QUEUE_CAPACITY; ++index) {
        entry = &plane->tx_queue[index];
        if (!adhoc_data_tx_entry_due(entry, now_us)) continue;
        if (!has_picked || entry->next_tx_us < plane->tx_queue[picked].next_tx_us ||
            (entry->next_tx_us == plane->tx_queue[picked].next_tx_us &&
             entry->retry_count < plane->tx_queue[picked].retry_count)) {
            picked = index;
            has_picked = 1u;
        }
    }
    if (!has_picked) return 0;
    *index_out = picked;
    return 1;
}

/** @brief 发送后更新待发条目: 增加重试计数, 若超过上限则推送 RETRY_EXHAUSTED 报告 */
static void adhoc_data_tx_entry_after_emit(adhoc_data_plane_t *plane, adhoc_data_tx_entry_t *entry, uint32_t now_us)
{
    adhoc_data_ack_item_t item;
    if (plane == 0 || entry == 0 || entry->used == 0u) return;
    entry->retry_count++;
    if (entry->retry_count >= plane->cfg.tx_retry_max) {
        item.source = entry->msg.source;
        item.lmt_d  = entry->msg.lmt_d;
        adhoc_data_tx_report_push(plane, ADHOC_DATA_TX_REPORT_RETRY_EXHAUSTED, &item, entry->retry_count);
        memset(entry, 0, sizeof(*entry));
        return;
    }
    entry->next_tx_us = now_us + plane->cfg.t5_us;
}

/** @brief 从当前时间计算时隙号低4位 */
static uint8_t adhoc_data_slot_from_time(uint32_t now_us, uint32_t t5_us)
{
    uint32_t cycle_idx;
    if (t5_us == 0u) return 0u;
    cycle_idx = now_us / t5_us;
    return (uint8_t)(cycle_idx & 0x0Fu);
}

/**
 * @brief 准备转发数据消息
 *
 * - 原发数据: 按发送者绑定号应用阈值判定(child_bind_no <= source.id_flag)
 * - 转发数据: 按发送者绑定号应用精确判定(child_bind_no == source.id_flag)
 * - 命中后统一改写为本节点指向上级的绑定号, 供下一跳按发送者视角继续判别
 */
static int adhoc_data_prepare_forward_msg(const adhoc_data_plane_t *plane, const adhoc_data_msg_t *rx_msg,
                                          const adhoc_data_downstream_binding_t *child_binding,
                                          adhoc_data_origin_t origin, adhoc_data_msg_t *tx_msg_out)
{
    uint8_t upstream_no;
    if (plane == 0 || rx_msg == 0 || tx_msg_out == 0 || child_binding == 0) return 0;
    if (!adhoc_data_msg_valid(rx_msg)) return 0;
    if (!adhoc_data_route_valid(plane)) return 0;
    if (!adhoc_data_bind_flag_forwardable(rx_msg->source.id_flag)) return 0;
    upstream_no = adhoc_data_local_upstream_no(plane);
    if (!adhoc_data_bind_flag_forwardable(upstream_no)) return 0;
    *tx_msg_out = *rx_msg;
    if (origin == ADHOC_DATA_ORIGIN_SOURCE) {
        if (child_binding->child_bind_no > rx_msg->source.id_flag) return 0;
        tx_msg_out->source.id_flag = upstream_no;
        return 1;
    }
    if (origin == ADHOC_DATA_ORIGIN_FORWARD && child_binding->child_bind_no == rx_msg->source.id_flag) {
        tx_msg_out->source.id_flag = upstream_no;
        return 1;
    }
    return 0;
}

/* ========== 去重表操作 ========== */

/** @brief 检查数据消息是否已在去重窗口内出现过 */
static int adhoc_data_dedup_is_duplicate(const adhoc_data_plane_t *plane, const adhoc_data_msg_t *msg, uint32_t now_ms)
{
    uint8_t index;
    const adhoc_data_dedup_entry_t *entry;
    if (plane == 0 || msg == 0) return 0;
    for (index = 0u; index < ADHOC_DATA_DEDUP_CAPACITY; ++index) {
        entry = &plane->dedup[index];
        if (!entry->used) continue;
        if (!adhoc_data_source_key_equal(&entry->source, entry->lmt_d, &msg->source, msg->lmt_d)) continue;
        if ((uint32_t)(now_ms - entry->last_seen_ms) <= plane->cfg.dedup_window_ms) return 1;
    }
    return 0;
}

/** @brief 将数据消息存入去重表(优先复用过期/空闲槽位) */
static void adhoc_data_dedup_store(adhoc_data_plane_t *plane, const adhoc_data_msg_t *msg, uint32_t now_ms)
{
    uint8_t index, target = 0u, has_target = 0u;
    uint32_t oldest_delta = 0u, delta;
    adhoc_data_dedup_entry_t *entry;
    if (plane == 0 || msg == 0) return;
    for (index = 0u; index < ADHOC_DATA_DEDUP_CAPACITY; ++index) {
        entry = &plane->dedup[index];
        if (!entry->used) { target = index; has_target = 1u; break; }
        if ((uint32_t)(now_ms - entry->last_seen_ms) > plane->cfg.dedup_window_ms) { target = index; has_target = 1u; break; }
        delta = (uint32_t)(now_ms - entry->last_seen_ms);
        if (!has_target || delta > oldest_delta) { oldest_delta = delta; target = index; has_target = 1u; }
    }
    if (!has_target) return;
    plane->dedup[target].used = 1u;
    plane->dedup[target].source = msg->source;
    plane->dedup[target].lmt_d  = msg->lmt_d;
    plane->dedup[target].last_seen_ms = now_ms;
}

/* ========== D帧/ACK 编解码 ========== */

/** @brief 打包数据消息为24B content: LMT_D[3] + source_id[4] + user[17] */
int adhoc_data_msg_pack(const adhoc_data_msg_t *msg, uint8_t content_out[ADHOC_FRAME_CONTENT_LEN])
{
    if (msg == 0 || content_out == 0) return 0;
    if (!adhoc_data_msg_valid(msg)) return 0;
    memset(content_out, 0, ADHOC_FRAME_CONTENT_LEN);
    adhoc_u24_be_write(msg->lmt_d, &content_out[0]);
    if (!adhoc_payload_id_pack(msg->source, &content_out[3])) return 0;
    memcpy(&content_out[7], msg->user, ADHOC_DATA_MSG_USER_LEN);
    return 1;
}

/** @brief 从24B content解出数据消息 */
int adhoc_data_msg_unpack(const uint8_t content[ADHOC_FRAME_CONTENT_LEN], adhoc_data_msg_t *msg_out)
{
    if (content == 0 || msg_out == 0) return 0;
    msg_out->lmt_d = adhoc_u24_be_read(&content[0]);
    if (!adhoc_payload_id_unpack(&content[3], &msg_out->source)) return 0;
    memcpy(msg_out->user, &content[7], ADHOC_DATA_MSG_USER_LEN);
    return 1;
}

/** @brief 打包ACK载荷: 每项7B(source_id[4]+lmt_d[3]), 最多3项 */
int adhoc_data_ack_payload_pack(const adhoc_data_ack_item_t *items, uint8_t item_count,
                                uint8_t content_out[ADHOC_FRAME_CONTENT_LEN])
{
    uint8_t index, limit;
    if (content_out == 0) return 0;
    memset(content_out, 0, ADHOC_FRAME_CONTENT_LEN);
    if (items == 0 && item_count != 0u) return 0;
    limit = item_count > ADHOC_DATA_ACK_MAX_PER_FRAME ? ADHOC_DATA_ACK_MAX_PER_FRAME : item_count;
    for (index = 0u; index < limit; ++index) {
        if (!adhoc_data_ack_item_valid(&items[index])) return 0;
        if (!adhoc_payload_id_pack(items[index].source, &content_out[index * 7u])) return 0;
        adhoc_u24_be_write(items[index].lmt_d, &content_out[index * 7u + 4u]);
    }
    return 1;
}

/** @brief 解包ACK载荷 */
int adhoc_data_ack_payload_unpack(const uint8_t content[ADHOC_FRAME_CONTENT_LEN],
                                  adhoc_data_ack_item_t items_out[ADHOC_DATA_ACK_MAX_PER_FRAME],
                                  uint8_t *item_count_out)
{
    uint8_t index, count = 0u;
    adhoc_data_ack_item_t item;
    if (content == 0 || items_out == 0) return 0;
    memset(items_out, 0, sizeof(adhoc_data_ack_item_t) * ADHOC_DATA_ACK_MAX_PER_FRAME);
    for (index = 0u; index < ADHOC_DATA_ACK_MAX_PER_FRAME; ++index) {
        if (!adhoc_payload_id_unpack(&content[index * 7u], &item.source)) continue;
        item.lmt_d = adhoc_u24_be_read(&content[index * 7u + 4u]);
        if (!adhoc_data_ack_item_valid(&item)) continue;
        items_out[count++] = item;
    }
    if (item_count_out != 0) *item_count_out = count;
    return 1;
}

/* ========== 数据面生命周期 ========== */

int adhoc_data_plane_init(adhoc_data_plane_t *plane, const adhoc_data_plane_cfg_t *cfg)
{
    if (plane == 0 || cfg == 0) return 0;
    if (cfg->domain_id > ADHOC_SENDER_DOMAIN_MAX || cfg->node_id == 0u || cfg->node_id > ADHOC_SENDER_NODE_MAX) return 0;
    if (cfg->gateway_no > 7u || cfg->t5_us == 0u) return 0;
    memset(plane, 0, sizeof(*plane));
    plane->cfg = *cfg;
    if (plane->cfg.dedup_window_ms == 0u) plane->cfg.dedup_window_ms = ADHOC_DATA_DEFAULT_DEDUP_WINDOW_MS;
    if (plane->cfg.tx_retry_max == 0u) plane->cfg.tx_retry_max = ADHOC_DATA_TX_RETRY_MAX;
    plane->role_gateway = cfg->role_gateway ? 1u : 0u;
    plane->inited = 1u;
    return 1;
}

int adhoc_data_plane_set_role(adhoc_data_plane_t *plane, uint8_t role_gateway)
{
    if (plane == 0 || plane->inited == 0u) return 0;
    plane->role_gateway = role_gateway ? 1u : 0u;
    plane->cfg.role_gateway = plane->role_gateway;
    plane->next_ack_tx_us = 0u;
    plane->ack_head = plane->ack_tail = plane->ack_count = 0u;
    memset(plane->ack_queue, 0, sizeof(plane->ack_queue));
    memset(plane->tx_queue, 0, sizeof(plane->tx_queue));
    memset(plane->tx_report_queue, 0, sizeof(plane->tx_report_queue));
    plane->tx_report_head = plane->tx_report_tail = plane->tx_report_count = 0u;
    plane->joined_level = plane->upstream_no = plane->upstream_gateway_no = 0u;
    plane->upstream_id = 0u;
    plane->downstream_binding_count = 0u;
    memset(plane->downstream_bindings, 0, sizeof(plane->downstream_bindings));
    plane->last_ack_hit_count = 0u;
    return 1;
}

/** @brief 设置数据面路由信息(由状态机同步) */
int adhoc_data_plane_set_route(adhoc_data_plane_t *plane, uint8_t joined_level, uint8_t upstream_gateway_no,
                               uint8_t upstream_no, uint32_t upstream_id,
                               const adhoc_data_downstream_binding_t *downstream_bindings,
                               uint8_t downstream_binding_count)
{
    uint8_t index, copy_count;
    if (plane == 0 || plane->inited == 0u) return 0;
    if (plane->role_gateway != 0u) {
        plane->joined_level = plane->upstream_no = plane->upstream_gateway_no = 0u;
        plane->upstream_id = 0u;
        plane->downstream_binding_count = 0u;
        memset(plane->downstream_bindings, 0, sizeof(plane->downstream_bindings));
        return 1;
    }
    if (joined_level == 0u) {
        plane->joined_level = plane->upstream_no = plane->upstream_gateway_no = 0u;
        plane->upstream_id = 0u;
        plane->downstream_binding_count = 0u;
        memset(plane->downstream_bindings, 0, sizeof(plane->downstream_bindings));
        return 1;
    }
    if (upstream_gateway_no > 7u || upstream_no > ADHOC_PAYLOAD_ID_FLAG_BIND_MAX || upstream_id == 0u) return 0;
    if (downstream_binding_count != 0u && downstream_bindings == 0) return 0;
    plane->joined_level = joined_level;
    plane->upstream_id   = upstream_id;
    plane->upstream_no   = upstream_no;
    plane->upstream_gateway_no = upstream_gateway_no;
    memset(plane->downstream_bindings, 0, sizeof(plane->downstream_bindings));
    copy_count = downstream_binding_count > ADHOC_DATA_DOWNSTREAM_BINDING_CAPACITY
               ? ADHOC_DATA_DOWNSTREAM_BINDING_CAPACITY : downstream_binding_count;
    for (index = 0u; index < copy_count; ++index)
        plane->downstream_bindings[index] = downstream_bindings[index];
    plane->downstream_binding_count = copy_count;
    return 1;
}

void adhoc_data_plane_reset(adhoc_data_plane_t *plane)
{
    if (plane == 0 || plane->inited == 0u) return;
    memset(plane->dedup, 0, sizeof(plane->dedup));
    memset(plane->ack_queue, 0, sizeof(plane->ack_queue));
    plane->ack_head = plane->ack_tail = plane->ack_count = 0u;
    plane->next_ack_tx_us = 0u;
    memset(plane->tx_queue, 0, sizeof(plane->tx_queue));
    memset(plane->tx_report_queue, 0, sizeof(plane->tx_report_queue));
    plane->tx_report_head = plane->tx_report_tail = plane->tx_report_count = 0u;
    plane->joined_level = plane->upstream_no = plane->upstream_gateway_no = 0u;
    plane->upstream_id = 0u;
    plane->downstream_binding_count = 0u;
    memset(plane->downstream_bindings, 0, sizeof(plane->downstream_bindings));
    plane->has_last_rx_data = 0u;
    plane->last_rx_duplicate = 0u;
    plane->last_rx_origin = ADHOC_DATA_ORIGIN_UNKNOWN;
    plane->last_ack_hit_count = 0u;
    plane->last_rx_ack_count = 0u;
    memset(&plane->last_rx_data, 0, sizeof(plane->last_rx_data));
    memset(plane->last_rx_acks, 0, sizeof(plane->last_rx_acks));
}

/** @brief 提交源数据(仅信标): 包装数据消息并推入待发队列 */
int adhoc_data_plane_submit_source_data(adhoc_data_plane_t *plane, uint8_t source_id_flag, uint32_t lmt_d,
                                        const uint8_t user[ADHOC_DATA_MSG_USER_LEN], uint8_t level, uint8_t gateway_no,
                                        uint32_t now_us)
{
    adhoc_data_msg_t msg;
    if (plane == 0 || user == 0 || plane->inited == 0u) return 0;
    if (plane->role_gateway != 0u) return 0;
    if (!adhoc_data_bind_flag_forwardable(source_id_flag)) return 0;
    memset(&msg, 0, sizeof(msg));
    msg.source.id_flag = source_id_flag;
    msg.source.node_id = plane->cfg.node_id;
    msg.lmt_d = lmt_d & ADHOC_DATA_LMT_D_MAX;
    memcpy(msg.user, user, ADHOC_DATA_MSG_USER_LEN);
    return adhoc_data_tx_queue_push_unique(plane, &msg, level, gateway_no, now_us);
}

int adhoc_data_plane_pop_tx_report(adhoc_data_plane_t *plane, adhoc_data_tx_report_t *out_report)
{
    if (plane == 0 || out_report == 0 || plane->inited == 0u) return 0;
    return adhoc_data_tx_report_pop(plane, out_report);
}

/**
 * @brief 数据面接收处理入口
 *
 * - D V=0(网关ACK): 解包ACK条目, 清理本地待发队列
 * - D V=n(数据): 去重 → 网关入ACK队列 / 信标入转发队列
 */
adhoc_data_rx_result_t adhoc_data_plane_on_rx(adhoc_data_plane_t *plane, const adhoc_frame_fields_t *fields, uint32_t ts_us)
{
    adhoc_data_msg_t data_msg, forward_msg;
    adhoc_data_ack_item_t ack_items[ADHOC_DATA_ACK_MAX_PER_FRAME], ack_item;
    const adhoc_data_downstream_binding_t *child_binding;
    uint8_t ack_count = 0u, ack_index;
    uint32_t now_ms;
    int is_duplicate;

    if (plane == 0 || fields == 0 || plane->inited == 0u) return ADHOC_DATA_RX_IGNORED;
    if (fields->msg_class != ADHOC_MSG_CLASS_D) return ADHOC_DATA_RX_IGNORED;
    if (fields->sender.domain_id != plane->cfg.domain_id) return ADHOC_DATA_RX_IGNORED;

    now_ms = adhoc_us_to_ms(ts_us);

    /* 网关 ACK 帧 */
    if (fields->level == 0u && fields->sender.node_id <= ADHOC_DATA_GATEWAY_ID_MAX) {
        if (!adhoc_data_ack_payload_unpack(fields->content, ack_items, &ack_count)) return ADHOC_DATA_RX_IGNORED;
        plane->last_ack_hit_count = 0u;
        plane->last_rx_ack_count = ack_count;
        memset(plane->last_rx_acks, 0, sizeof(plane->last_rx_acks));
        if (ack_count != 0u) {
            memcpy(plane->last_rx_acks, ack_items, sizeof(adhoc_data_ack_item_t) * ack_count);
            for (ack_index = 0u; ack_index < ack_count; ++ack_index)
                plane->last_ack_hit_count += adhoc_data_tx_queue_remove_by_ack(plane, &ack_items[ack_index]);
        }
        return ADHOC_DATA_RX_ACK;
    }

    /* 数据帧 */
    if (!adhoc_data_msg_unpack(fields->content, &data_msg)) return ADHOC_DATA_RX_IGNORED;
    plane->has_last_rx_data = 1u;
    plane->last_rx_data = data_msg;
    plane->last_rx_origin = (fields->sender.node_id == data_msg.source.node_id) ? ADHOC_DATA_ORIGIN_SOURCE : ADHOC_DATA_ORIGIN_FORWARD;
    child_binding = adhoc_data_find_downstream_binding(plane, fields);

    /* 监听确认: 仅收到直接上级的转发数据时，移除本地匹配待发项 */
    if (plane->role_gateway == 0u && plane->last_rx_origin == ADHOC_DATA_ORIGIN_FORWARD &&
        fields->sender.node_id != plane->cfg.node_id &&
        adhoc_data_sender_is_upstream(plane, fields))
        (void)adhoc_data_tx_queue_remove_by_forward(plane, &data_msg);

    /* 去重 */
    is_duplicate = adhoc_data_dedup_is_duplicate(plane, &data_msg, now_ms);
    plane->last_rx_duplicate = is_duplicate ? 1u : 0u;
    if (!is_duplicate) adhoc_data_dedup_store(plane, &data_msg, now_ms);

    /* 网关: 生成ACK条目 */
    if (plane->role_gateway != 0u) {
        ack_item.source = data_msg.source;
        ack_item.lmt_d  = data_msg.lmt_d;
        (void)adhoc_data_ack_queue_push_unique(plane, &ack_item);
        if (plane->next_ack_tx_us == 0u) plane->next_ack_tx_us = ts_us;
    }
    /* 信标: 准备转发 */
    else if (!is_duplicate && fields->sender.node_id != plane->cfg.node_id) {
        if (adhoc_data_prepare_forward_msg(plane, &data_msg, child_binding, plane->last_rx_origin, &forward_msg))
            (void)adhoc_data_tx_queue_push_unique(plane, &forward_msg, plane->joined_level, plane->upstream_gateway_no, ts_us);
    }

    return is_duplicate ? ADHOC_DATA_RX_DUPLICATE : ADHOC_DATA_RX_ACCEPTED;
}

/** @brief 网关 ACK 发送调度: 从ACK队列取最多3条打包发送 */
int adhoc_data_plane_poll_gateway_ack(adhoc_data_plane_t *plane, uint32_t now_us, adhoc_frame_fields_t *out_fields, uint8_t *out_has_tx)
{
    adhoc_data_ack_item_t items[ADHOC_DATA_ACK_MAX_PER_FRAME];
    uint8_t count = 0u;
    if (plane == 0 || out_has_tx == 0 || plane->inited == 0u) return 0;
    *out_has_tx = 0u;
    if (plane->role_gateway == 0u || plane->ack_count == 0u) return 1;
    if (plane->next_ack_tx_us != 0u && now_us < plane->next_ack_tx_us) return 1;
    if (out_fields == 0) return 0;
    memset(out_fields, 0, sizeof(*out_fields));
    out_fields->msg_class  = ADHOC_MSG_CLASS_D;
    out_fields->gateway_no = plane->cfg.gateway_no;
    out_fields->slot_high4 = 0u;
    out_fields->level      = 0u;
    out_fields->sender.domain_id = plane->cfg.domain_id;
    out_fields->sender.node_id   = plane->cfg.node_id;
    while (count < ADHOC_DATA_ACK_MAX_PER_FRAME) { if (!adhoc_data_ack_queue_pop(plane, &items[count])) break; count++; }
    if (!adhoc_data_ack_payload_pack(items, count, out_fields->content)) return 0;
    *out_has_tx = 1u;
    plane->next_ack_tx_us = now_us + plane->cfg.t5_us;
    return 1;
}

/** @brief 数据转发发送调度: 从待发队列选取到期条目发送 */
int adhoc_data_plane_poll_forward_tx(adhoc_data_plane_t *plane, uint32_t now_us,
                                     adhoc_frame_fields_t *out_fields, uint8_t *out_has_tx)
{
    uint8_t tx_index;
    adhoc_data_tx_entry_t *entry;
    if (plane == 0 || out_has_tx == 0 || plane->inited == 0u) return 0;
    *out_has_tx = 0u;
    if (plane->role_gateway != 0u) return 1;
    if (out_fields == 0) return 0;
    if (!adhoc_data_tx_queue_pick_due(plane, now_us, &tx_index)) return 1;
    entry = &plane->tx_queue[tx_index];
    if (!entry->used) return 1;
    memset(out_fields, 0, sizeof(*out_fields));
    out_fields->msg_class  = ADHOC_MSG_CLASS_D;
    out_fields->gateway_no = entry->gateway_no;
    out_fields->slot_high4 = adhoc_data_slot_from_time(now_us, plane->cfg.t5_us);
    out_fields->level      = entry->level;
    out_fields->sender.domain_id = plane->cfg.domain_id;
    out_fields->sender.node_id   = plane->cfg.node_id;
    if (!adhoc_data_msg_pack(&entry->msg, out_fields->content)) return 0;
    *out_has_tx = 1u;
    adhoc_data_tx_entry_after_emit(plane, entry, now_us);
    return 1;
}
