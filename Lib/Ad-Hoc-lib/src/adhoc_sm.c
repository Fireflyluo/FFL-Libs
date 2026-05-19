/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_sm.c
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            组网状态机实现
**                          管理 ST1/U1/UN/C1/CN 状态迁移, 网关 T2 收集窗口,
**                          邻居缓存(Top-K)、上级绑定表、下行确认队列
**--------------------------------------------------------------------------------------------------------
*/

#include "adhoc_sm.h"

#include <string.h>

#ifndef ADHOC_TEST_DROP_JOIN_CONFIRM
#define ADHOC_TEST_DROP_JOIN_CONFIRM ADHOC_CONFIG_TEST_DROP_JOIN_CONFIRM
#endif

/* ========== 邻居/候选辅助函数 ========== */

/** @brief 清空候选邻居 */
static void adhoc_sm_candidate_clear(adhoc_sm_candidate_t *candidate)
{
    if (candidate == 0) return;
    memset(candidate, 0, sizeof(*candidate));
}

/** @brief 清空邻居缓存 */
static void adhoc_sm_neighbor_cache_reset(adhoc_sm_t *sm)
{
    if (sm == 0) return;
    memset(sm->neighbor_cache, 0, sizeof(sm->neighbor_cache));
}

/** @brief 清空上级绑定表 */
static void adhoc_sm_upstream_bindings_reset(adhoc_sm_t *sm)
{
    if (sm == 0) return;
    memset(sm->upstream_bindings, 0, sizeof(sm->upstream_bindings));
}

/** @brief 清空直接下级绑定记录 */
static void adhoc_sm_downstream_bindings_reset(adhoc_sm_t *sm)
{
    if (sm == 0) return;
    memset(sm->downstream_bindings, 0, sizeof(sm->downstream_bindings));
}

/** @brief 查找直接下级绑定槽位 */
static int adhoc_sm_downstream_binding_find_slot(const adhoc_sm_t *sm, uint32_t child_id)
{
    uint8_t index;
    if (sm == 0 || child_id == 0u) return -1;
    for (index = 0u; index < ADHOC_SM_DOWNSTREAM_BINDING_CAPACITY; ++index) {
        if (sm->downstream_bindings[index].used == 0u) continue;
        if (sm->downstream_bindings[index].child_id == child_id) return (int)index;
    }
    return -1;
}

/** @brief 选择直接下级绑定写入槽位(优先空闲, 否则替换最久未见) */
static int adhoc_sm_downstream_binding_choose_slot(const adhoc_sm_t *sm)
{
    uint8_t index;
    int free_index = -1, oldest_index = -1;
    if (sm == 0) return -1;
    for (index = 0u; index < ADHOC_SM_DOWNSTREAM_BINDING_CAPACITY; ++index) {
        if (sm->downstream_bindings[index].used == 0u) {
            if (free_index < 0) free_index = (int)index;
            continue;
        }
        if (oldest_index < 0 || sm->downstream_bindings[index].last_seen_us < sm->downstream_bindings[oldest_index].last_seen_us)
            oldest_index = (int)index;
    }
    if (free_index >= 0) return free_index;
    return oldest_index;
}

/** @brief 插入或更新直接下级绑定记录 */
static void adhoc_sm_downstream_binding_upsert(adhoc_sm_t *sm, uint32_t child_id, uint8_t child_bind_no,
                                               uint8_t child_level, uint8_t gateway_no, uint32_t ts_us)
{
    int slot;
    adhoc_sm_downstream_binding_t *binding;
    if (sm == 0 || child_id == 0u || child_bind_no > ADHOC_PAYLOAD_ID_FLAG_BIND_MAX) return;
    slot = adhoc_sm_downstream_binding_find_slot(sm, child_id);
    if (slot < 0) slot = adhoc_sm_downstream_binding_choose_slot(sm);
    if (slot < 0) return;
    binding = &sm->downstream_bindings[slot];
    memset(binding, 0, sizeof(*binding));
    binding->used = 1u;
    binding->child_bind_no = child_bind_no;
    binding->child_level = child_level;
    binding->gateway_no = gateway_no;
    binding->child_id = child_id;
    binding->last_seen_us = ts_us;
}

/** @brief 刷新直接下级最近观测时间 */
static void adhoc_sm_downstream_binding_touch(adhoc_sm_t *sm, uint32_t child_id, uint8_t child_level,
                                              uint8_t gateway_no, uint32_t ts_us)
{
    int slot;
    adhoc_sm_downstream_binding_t *binding;
    if (sm == 0 || child_id == 0u) return;
    slot = adhoc_sm_downstream_binding_find_slot(sm, child_id);
    if (slot < 0) return;
    binding = &sm->downstream_bindings[slot];
    binding->child_level = child_level;
    binding->gateway_no = gateway_no;
    binding->last_seen_us = ts_us;
}

/** @brief 释放指定上级绑定条目 */
static void adhoc_sm_release_upstream_binding(adhoc_sm_t *sm, uint8_t gateway_no, uint8_t upstream_level, uint32_t upstream_id)
{
    uint8_t index;
    adhoc_sm_upstream_binding_t *binding;
    if (sm == 0 || upstream_id == 0u) return;
    for (index = 0u; index < ADHOC_SM_UPSTREAM_BINDING_CAPACITY; ++index) {
        binding = &sm->upstream_bindings[index];
        if (binding->used == 0u) continue;
        if (binding->gateway_no == gateway_no && binding->upstream_level == upstream_level &&
            binding->upstream_id == upstream_id) {
            memset(binding, 0, sizeof(*binding));
            return;
        }
    }
}

/** @brief 清理邻居(含释放对应的上级绑定) */
static void adhoc_sm_neighbor_clear(adhoc_sm_t *sm, adhoc_sm_candidate_t *neighbor)
{
    if (neighbor == 0) return;
    if (sm != 0 && neighbor->active != 0u)
        adhoc_sm_release_upstream_binding(sm, neighbor->gateway_no, neighbor->upstream_level, neighbor->upstream_id);
    adhoc_sm_candidate_clear(neighbor);
}

/** @brief 为上级分配或查找绑定编号(0~5), 空闲槽位中选择未使用的编号 */
static uint8_t adhoc_sm_pick_upstream_no(adhoc_sm_t *sm, uint8_t gateway_no, uint8_t upstream_level, uint32_t upstream_id)
{
    uint8_t index, free_index = 0xFFu, used_no_mask = 0u;
    adhoc_sm_upstream_binding_t *binding;
    if (sm == 0 || upstream_id == 0u || gateway_no > 7u || upstream_level > ADHOC_SM_JOIN_LEVEL_MAX) return 0u;
    for (index = 0u; index < ADHOC_SM_UPSTREAM_BINDING_CAPACITY; ++index) {
        binding = &sm->upstream_bindings[index];
        if (binding->used == 0u) { if (free_index == 0xFFu) free_index = index; continue; }
        if (binding->upstream_no < ADHOC_SM_UPSTREAM_BINDING_CAPACITY)
            used_no_mask |= (uint8_t)(1u << binding->upstream_no);
        if (binding->gateway_no == gateway_no && binding->upstream_level == upstream_level &&
            binding->upstream_id == upstream_id) return binding->upstream_no;
    }
    if (free_index != 0xFFu) {
        for (index = 0u; index < ADHOC_SM_UPSTREAM_BINDING_CAPACITY; ++index) {
            if ((used_no_mask & (uint8_t)(1u << index)) == 0u) {
                binding = &sm->upstream_bindings[free_index];
                binding->used = 1u; binding->upstream_no = index;
                binding->gateway_no = gateway_no; binding->upstream_level = upstream_level;
                binding->upstream_id = upstream_id;
                return index;
            }
        }
    }
    return 0u;
}

/** @brief RSSI → 信号等级: >=-65强(3), >=-80中(2), <-80弱(1) */
static uint8_t adhoc_sm_rssi_rank(int8_t rssi)
{
    if (rssi >= ADHOC_SM_RSSI_STRONG_MIN_DBM) return 3u;
    if (rssi >= ADHOC_SM_RSSI_MEDIUM_MIN_DBM) return 2u;
    return 1u;
}

/** @brief 候选邻居比较: 按 signal_rank→RSSI→upstream_level(越小越好)→gateway_no→upstream_id */
static int adhoc_sm_candidate_cmp_key(uint8_t left_signal_rank, int8_t left_rssi,
                                      uint8_t left_upstream_level, uint8_t left_gateway_no, uint32_t left_upstream_id,
                                      uint8_t right_signal_rank, int8_t right_rssi,
                                      uint8_t right_upstream_level, uint8_t right_gateway_no, uint32_t right_upstream_id)
{
    if (left_signal_rank != right_signal_rank) return left_signal_rank > right_signal_rank ? 1 : -1;
    if (left_rssi != right_rssi) return left_rssi > right_rssi ? 1 : -1;
    if (left_upstream_level != right_upstream_level) return left_upstream_level < right_upstream_level ? 1 : -1;
    if (left_gateway_no != right_gateway_no) return left_gateway_no < right_gateway_no ? 1 : -1;
    if (left_upstream_id != right_upstream_id) return left_upstream_id < right_upstream_id ? 1 : -1;
    return 0;
}

/** @brief 根据信号等级确定命中要求和窗口周期: 强(1/1), 中(1/2), 弱(2/4) */
static void adhoc_sm_candidate_rule(uint8_t signal_rank, uint8_t *required_hits, uint8_t *window_cycles)
{
    if (required_hits == 0 || window_cycles == 0) return;
    if (signal_rank >= 3u)       { *required_hits = 1u; *window_cycles = 1u; }
    else if (signal_rank == 2u)  { *required_hits = 1u; *window_cycles = 2u; }
    else                         { *required_hits = 2u; *window_cycles = 4u; }
}

/** @brief 从微秒时间戳计算 T5 周期索引 */
static uint32_t adhoc_sm_cycle_index(uint32_t ts_us, uint32_t t5_us)
{
    return t5_us == 0u ? 0u : ts_us / t5_us;
}

/** @brief 检查 now_us 是否已达到 deadline_us(处理回绕) */
static int adhoc_sm_time_reached(uint32_t now_us, uint32_t deadline_us)
{
    return (uint32_t)(now_us - deadline_us) < 0x80000000u ? 1 : 0;
}

/** @brief 上级丢失超时: T5 * ADHOC_SM_UPSTREAM_LOSS_CYCLES (默认 3*T5) */
static uint32_t adhoc_sm_upstream_loss_timeout_us(const adhoc_sm_t *sm)
{
    if (sm == 0) return 0u;
    return sm->cfg.t5_us * ADHOC_SM_UPSTREAM_LOSS_CYCLES;
}

/** @brief 检查候选邻居是否匹配指定上级 */
static int adhoc_sm_candidate_matches(const adhoc_sm_candidate_t *candidate, uint8_t gateway_no, uint8_t upstream_level, uint32_t upstream_id)
{
    if (candidate == 0 || candidate->active == 0u) return 0;
    return candidate->gateway_no == gateway_no && candidate->upstream_level == upstream_level &&
           candidate->upstream_id == upstream_id;
}

/** @brief 初始化候选邻居条目 */
static void adhoc_sm_candidate_init(adhoc_sm_t *sm, adhoc_sm_candidate_t *candidate,
                                    uint8_t signal_rank, int8_t rssi,
                                    uint8_t gateway_no, uint8_t upstream_level, uint32_t upstream_id,
                                    uint32_t ts_us, uint32_t cycle_idx)
{
    uint8_t required_hits = 0u, window_cycles = 0u;
    if (sm == 0 || candidate == 0) return;
    adhoc_sm_candidate_rule(signal_rank, &required_hits, &window_cycles);
    memset(candidate, 0, sizeof(*candidate));
    candidate->active = 1u;
    candidate->upstream_level = upstream_level;
    candidate->upstream_no = adhoc_sm_pick_upstream_no(sm, gateway_no, upstream_level, upstream_id);
    candidate->gateway_no  = gateway_no;
    candidate->upstream_id = upstream_id;
    candidate->signal_rank   = signal_rank;
    candidate->required_hits  = required_hits;
    candidate->window_cycles  = window_cycles;
    candidate->last_rssi      = rssi;
    candidate->first_cycle_idx = cycle_idx;
    candidate->last_seen_us   = ts_us;
    candidate->network_end_us = ts_us + sm->cfg.network_window_us;
}

/** @brief 更新候选邻居的命中记录(同周期不重复计数) */
static void adhoc_sm_candidate_touch(adhoc_sm_candidate_t *candidate, uint8_t signal_rank, int8_t rssi,
                                     uint32_t ts_us, uint32_t cycle_idx)
{
    uint8_t required_hits = 0u, window_cycles = 0u;
    if (candidate == 0 || candidate->active == 0u) return;
    adhoc_sm_candidate_rule(signal_rank, &required_hits, &window_cycles);
    candidate->signal_rank   = signal_rank;
    candidate->required_hits  = required_hits;
    candidate->window_cycles  = window_cycles;
    candidate->last_rssi      = rssi;
    candidate->last_seen_us   = ts_us;
    if (candidate->has_last_hit_cycle == 0u || cycle_idx != candidate->last_hit_cycle_idx) {
        candidate->last_hit_cycle_idx = cycle_idx;
        candidate->has_last_hit_cycle = 1u;
        if (candidate->success_hits < 0xFFu) candidate->success_hits++;
    }
}

/** @brief 检查候选邻居是否已过期(超出评估窗口或网络窗口到期) */
static int adhoc_sm_candidate_is_expired(const adhoc_sm_candidate_t *candidate, uint32_t cycle_idx, uint32_t now_us)
{
    uint32_t elapsed_cycles, idle_cycles;
    if (candidate == 0 || candidate->active == 0u) return 1;
    if (candidate->network_end_us != 0u && adhoc_sm_time_reached(now_us, candidate->network_end_us)) return 1;
    if (cycle_idx < candidate->first_cycle_idx) return 1;
    elapsed_cycles = cycle_idx - candidate->first_cycle_idx + 1u;
    if (candidate->success_hits < candidate->required_hits && elapsed_cycles > candidate->window_cycles) return 1;
    if (candidate->has_last_hit_cycle == 0u) return 0;
    if (cycle_idx < candidate->last_hit_cycle_idx) return 1;
    idle_cycles = cycle_idx - candidate->last_hit_cycle_idx + 1u;
    return idle_cycles > candidate->window_cycles ? 1 : 0;
}

/** @brief 检查候选邻居是否满足入网条件(未过期且命中次数达标) */
static int adhoc_sm_candidate_is_ready(const adhoc_sm_candidate_t *candidate, uint32_t cycle_idx, uint32_t now_us)
{
    if (adhoc_sm_candidate_is_expired(candidate, cycle_idx, now_us)) return 0;
    return candidate->success_hits >= candidate->required_hits ? 1 : 0;
}

/** @brief 扫描并清理过期邻居 */
static void adhoc_sm_neighbors_sweep(adhoc_sm_t *sm, uint32_t cycle_idx, uint32_t now_us)
{
    uint8_t index;
    if (sm == 0) return;
    for (index = 0u; index < ADHOC_SM_NEIGHBOR_CACHE_CAPACITY; ++index)
        if (adhoc_sm_candidate_is_expired(&sm->neighbor_cache[index], cycle_idx, now_us))
            adhoc_sm_neighbor_clear(sm, &sm->neighbor_cache[index]);
}

/** @brief 检查时间戳是否在网关 T2 接收窗口内 */
static int adhoc_sm_gateway_in_rx_window(const adhoc_sm_t *sm, uint32_t ts_us)
{
    if (sm == 0 || sm->gateway_rx_window_open == 0u) return 0;
    return (uint32_t)(ts_us - sm->gateway_rx_window_start_us) <= sm->cfg.t2_us ? 1 : 0;
}

/** @brief 生成与 level 奇偶匹配的时隙号(随机种子低4位适配奇偶) */
static uint8_t adhoc_sm_slot_match_level(uint8_t seed, uint8_t level)
{
    uint8_t slot = (uint8_t)(seed & 0x0Fu);
    if (((slot ^ level) & 0x01u) != 0u) slot ^= 0x01u;
    return slot;
}

/** @brief 判断新候选是否优于当前候选 */
static int adhoc_sm_is_better_candidate(const adhoc_sm_candidate_t *current,
                                        uint8_t signal_rank, int8_t rssi, uint8_t upstream_level,
                                        uint8_t gateway_no, uint32_t upstream_id)
{
    if (current == 0 || current->active == 0u) return 1;
    return adhoc_sm_candidate_cmp_key(signal_rank, rssi, upstream_level, gateway_no, upstream_id,
                                      current->signal_rank, current->last_rssi,
                                      current->upstream_level, current->gateway_no, current->upstream_id) > 0 ? 1 : 0;
}

/** @brief 在邻居缓存中查找匹配的槽位 */
static int adhoc_sm_neighbor_find_slot(const adhoc_sm_t *sm, uint8_t gateway_no, uint8_t upstream_level, uint32_t upstream_id)
{
    uint8_t index;
    if (sm == 0) return -1;
    for (index = 0u; index < ADHOC_SM_NEIGHBOR_CACHE_CAPACITY; ++index)
        if (adhoc_sm_candidate_matches(&sm->neighbor_cache[index], gateway_no, upstream_level, upstream_id))
            return (int)index;
    return -1;
}

/** @brief 在邻居缓存中选择存放新候选的槽位(优先空闲, 其次替换最差的) */
static int adhoc_sm_neighbor_choose_slot(adhoc_sm_t *sm, uint8_t signal_rank, int8_t rssi,
                                         uint8_t gateway_no, uint8_t upstream_level, uint32_t upstream_id)
{
    uint8_t index;
    int free_index = -1, worst_index = -1;
    adhoc_sm_candidate_t *neighbor;
    if (sm == 0) return -1;
    for (index = 0u; index < ADHOC_SM_NEIGHBOR_CACHE_CAPACITY; ++index) {
        neighbor = &sm->neighbor_cache[index];
        if (neighbor->active == 0u) { if (free_index < 0) free_index = (int)index; continue; }
        if (worst_index < 0 ||
            adhoc_sm_candidate_cmp_key(neighbor->signal_rank, neighbor->last_rssi,
                                       neighbor->upstream_level, neighbor->gateway_no, neighbor->upstream_id,
                                       sm->neighbor_cache[worst_index].signal_rank,
                                       sm->neighbor_cache[worst_index].last_rssi,
                                       sm->neighbor_cache[worst_index].upstream_level,
                                       sm->neighbor_cache[worst_index].gateway_no,
                                       sm->neighbor_cache[worst_index].upstream_id) < 0)
            worst_index = (int)index;
    }
    if (free_index >= 0) return free_index;
    if (worst_index < 0) return -1;
    if (!adhoc_sm_is_better_candidate(&sm->neighbor_cache[worst_index], signal_rank, rssi, upstream_level, gateway_no, upstream_id))
        return -1;
    adhoc_sm_neighbor_clear(sm, &sm->neighbor_cache[worst_index]);
    return worst_index;
}

/** @brief 释放除指定上级外的所有邻居(方向锁定后清理) */
static void adhoc_sm_neighbors_release_except(adhoc_sm_t *sm, uint8_t gateway_no, uint8_t upstream_level, uint32_t upstream_id)
{
    uint8_t index;
    adhoc_sm_candidate_t *neighbor;
    if (sm == 0) return;
    for (index = 0u; index < ADHOC_SM_NEIGHBOR_CACHE_CAPACITY; ++index) {
        neighbor = &sm->neighbor_cache[index];
        if (neighbor->active == 0u) continue;
        if (adhoc_sm_candidate_matches(neighbor, gateway_no, upstream_level, upstream_id))
            adhoc_sm_candidate_clear(neighbor);
        else
            adhoc_sm_neighbor_clear(sm, neighbor);
    }
}

/** @brief 记录接收到的 A 帧到邻居缓存(更新或新建候选) */
static int adhoc_sm_neighbor_record(adhoc_sm_t *sm, const adhoc_sm_rx_event_t *event, uint8_t signal_rank)
{
    int slot;
    uint32_t cycle_idx;
    adhoc_sm_candidate_t *neighbor;
    if (sm == 0 || event == 0) return 0;
    cycle_idx = adhoc_sm_cycle_index(event->ts_us, sm->cfg.t5_us);
    adhoc_sm_neighbors_sweep(sm, cycle_idx, event->ts_us);
    slot = adhoc_sm_neighbor_find_slot(sm, event->gateway_no, event->level, event->sender.node_id);
    if (slot < 0) {
        slot = adhoc_sm_neighbor_choose_slot(sm, signal_rank, event->rssi, event->gateway_no, event->level, event->sender.node_id);
        if (slot < 0) return 0;
        adhoc_sm_candidate_init(sm, &sm->neighbor_cache[slot], signal_rank, event->rssi,
                                event->gateway_no, event->level, event->sender.node_id, event->ts_us, cycle_idx);
    }
    neighbor = &sm->neighbor_cache[slot];
    if (cycle_idx < neighbor->first_cycle_idx) {
        adhoc_sm_neighbor_clear(sm, neighbor);
        adhoc_sm_candidate_init(sm, neighbor, signal_rank, event->rssi,
                                event->gateway_no, event->level, event->sender.node_id, event->ts_us, cycle_idx);
    }
    adhoc_sm_candidate_touch(neighbor, signal_rank, event->rssi, event->ts_us, cycle_idx);
    return 1;
}

/** @brief 从邻居缓存中选择最优的已就绪候选 */
static int adhoc_sm_select_best_ready_neighbor(adhoc_sm_t *sm, uint32_t ts_us)
{
    uint8_t index, found = 0u;
    uint32_t cycle_idx;
    adhoc_sm_candidate_t *neighbor;
    if (sm == 0) return 0;
    cycle_idx = adhoc_sm_cycle_index(ts_us, sm->cfg.t5_us);
    adhoc_sm_neighbors_sweep(sm, cycle_idx, ts_us);
    adhoc_sm_candidate_clear(&sm->candidate);
    for (index = 0u; index < ADHOC_SM_NEIGHBOR_CACHE_CAPACITY; ++index) {
        neighbor = &sm->neighbor_cache[index];
        if (!adhoc_sm_candidate_is_ready(neighbor, cycle_idx, ts_us)) continue;
        if (found == 0u || adhoc_sm_is_better_candidate(&sm->candidate, neighbor->signal_rank, neighbor->last_rssi,
                                                         neighbor->upstream_level, neighbor->gateway_no, neighbor->upstream_id)) {
            sm->candidate = *neighbor;
            found = 1u;
        }
    }
    return found != 0u ? 1 : 0;
}

/** @brief 检查发送者 node_id 是否符合 level 对应的 ID 范围(level=0→网关ID, level≥1→信标ID) */
static int adhoc_sm_sender_matches_level(uint8_t upstream_level, uint32_t sender_id)
{
    if (sender_id == 0u || sender_id > ADHOC_SENDER_NODE_MAX) return 0;
    if (upstream_level == 0u) return sender_id <= ADHOC_SM_GATEWAY_ID_MAX ? 1 : 0;
    return sender_id >= ADHOC_SM_BEACON_ID_MIN ? 1 : 0;
}

/** @brief 计算 A 帧 content 中第 N 个 ID 槽位的字节偏移 */
static uint8_t adhoc_sm_content_id_offset(uint8_t slot_idx)
{
    return (uint8_t)(ADHOC_A_CONFIRM_PAYLOAD_OFFSET + slot_idx * ADHOC_REPLY_ITEM_ENCODED_LEN);
}

/** @brief 向 content 写入 LMT_A 时间戳(4字节大端) */
static void adhoc_sm_content_write_lmt_a(uint8_t content[ADHOC_FRAME_CONTENT_LEN], uint32_t now_us)
{
    if (content == 0) return;
    adhoc_u32_be_write(adhoc_lmt_a_from_us(now_us), &content[0]);
}

/** @brief 扫描 content 中是否有 flag=7 且 node_id==self_node_id 的确认条目 */
static int adhoc_sm_content_has_confirm_self(const uint8_t content[ADHOC_FRAME_CONTENT_LEN], uint32_t self_node_id)
{
    uint8_t index;
    adhoc_payload_id_t packed_id;
    if (content == 0 || self_node_id == 0u) return 0;
    for (index = 0u; index < ADHOC_REPLY_MAX_PER_FRAME; ++index) {
        if (!adhoc_payload_id_unpack(&content[adhoc_sm_content_id_offset(index)], &packed_id)) continue;
        if (packed_id.id_flag == ADHOC_PAYLOAD_ID_FLAG_JOIN_CONFIRM && packed_id.node_id == self_node_id) return 1;
    }
    return 0;
}

/** @brief 扫描 content 中是否有 id_flag 在 [flag_min, flag_max] 范围内且 node_id==target_id 的条目 */
static int adhoc_sm_content_has_target_flag_range(const uint8_t content[ADHOC_FRAME_CONTENT_LEN], uint32_t target_id,
                                                  uint8_t flag_min, uint8_t flag_max)
{
    uint8_t index;
    adhoc_payload_id_t packed_id;
    if (content == 0 || target_id == 0u || flag_min > flag_max || flag_max > ADHOC_PAYLOAD_ID_FLAG_MAX) return 0;
    for (index = 0u; index < ADHOC_REPLY_MAX_PER_FRAME; ++index) {
        if (!adhoc_payload_id_unpack(&content[adhoc_sm_content_id_offset(index)], &packed_id)) continue;
        if (packed_id.node_id == target_id && packed_id.id_flag >= flag_min && packed_id.id_flag <= flag_max) return 1;
    }
    return 0;
}

/** @brief 读取指向本节点的绑定编号(flag=0~5) */
static int adhoc_sm_content_find_target_bind_no(const uint8_t content[ADHOC_FRAME_CONTENT_LEN], uint32_t target_id,
                                                uint8_t *bind_no_out)
{
    uint8_t index;
    adhoc_payload_id_t packed_id;
    if (content == 0 || bind_no_out == 0 || target_id == 0u) return 0;
    for (index = 0u; index < ADHOC_REPLY_MAX_PER_FRAME; ++index) {
        if (!adhoc_payload_id_unpack(&content[adhoc_sm_content_id_offset(index)], &packed_id)) continue;
        if (packed_id.node_id != target_id) continue;
        if (packed_id.id_flag > ADHOC_PAYLOAD_ID_FLAG_BIND_MAX) continue;
        *bind_no_out = packed_id.id_flag;
        return 1;
    }
    return 0;
}

/** @brief 向 content 指定槽位写入 ID */
static int adhoc_sm_content_write_id(uint8_t content[ADHOC_FRAME_CONTENT_LEN], uint8_t slot_idx,
                                     uint8_t id_flag, uint32_t node_id)
{
    adhoc_payload_id_t packed_id;
    if (content == 0 || slot_idx >= ADHOC_REPLY_MAX_PER_FRAME) return 0;
    packed_id.id_flag = id_flag;
    packed_id.node_id = node_id;
    return adhoc_payload_id_pack(packed_id, &content[adhoc_sm_content_id_offset(slot_idx)]);
}

/** @brief 判断是否为未确认状态(U1/UN) */
static int adhoc_sm_is_unconfirmed_state(adhoc_sm_state_t state)
{
    return state == ADHOC_SM_STATE_U1 || state == ADHOC_SM_STATE_UN;
}

/** @brief 判断是否为已确认状态(C1/CN) */
static int adhoc_sm_is_confirmed_state(adhoc_sm_state_t state)
{
    return state == ADHOC_SM_STATE_C1 || state == ADHOC_SM_STATE_CN;
}

/** @brief 判断接收事件是否来自当前锁定上级 */
static int adhoc_sm_event_is_current_upstream(const adhoc_sm_t *sm, const adhoc_sm_rx_event_t *event)
{
    if (sm == 0 || event == 0 || sm->joined_level == 0u) return 0;
    if (event->msg_class != ADHOC_MSG_CLASS_A) return 0;
    if (event->gateway_no != sm->upstream_gateway_no) return 0;
    if (event->sender.node_id != sm->upstream_id) return 0;
    return event->level + 1u == sm->joined_level ? 1 : 0;
}

/* ========== A 帧发射函数 ========== */

/** @brief 组装入网请求 A 帧(未确认态): LMT_A + upstream_id(flag=绑定编号) + 确认列表 */
static int adhoc_sm_emit_join_response(adhoc_sm_t *sm, uint8_t slot_seed, uint32_t now_us, adhoc_frame_fields_t *out_fields)
{
    adhoc_payload_id_t packed_id;
    if (sm == 0 || out_fields == 0 || sm->upstream_id == 0u || sm->joined_level == 0u) return 0;
    memset(out_fields, 0, sizeof(*out_fields));
    out_fields->msg_class  = ADHOC_MSG_CLASS_A;
    out_fields->gateway_no = sm->upstream_gateway_no;
    out_fields->slot_high4 = adhoc_sm_slot_match_level(slot_seed, sm->joined_level);
    out_fields->level      = sm->joined_level;
    out_fields->sender.domain_id = sm->cfg.domain_id;
    out_fields->sender.node_id   = sm->cfg.node_id;
    adhoc_sm_content_write_lmt_a(out_fields->content, now_us);
    packed_id.id_flag = sm->upstream_no;
    packed_id.node_id = sm->upstream_id;
    if (!adhoc_payload_id_pack(packed_id, &out_fields->content[ADHOC_A_CONFIRM_PAYLOAD_OFFSET])) return 0;
    if (!adhoc_reply_list_build_confirm_payload(&sm->downlink_confirm_list, out_fields->content,
                                                (uint8_t)(ADHOC_A_CONFIRM_PAYLOAD_OFFSET + ADHOC_REPLY_ITEM_ENCODED_LEN),
                                                ADHOC_REPLY_MAX_PER_FRAME - 1u, 0)) return 0;
    return 1;
}

/** @brief 组装已确认信标广播 A 帧: LMT_A + 已确认上级(flag=6) + 确认列表(flag=7)×4 */
static int adhoc_sm_emit_level_beacon(adhoc_sm_t *sm, uint8_t slot_seed, uint32_t now_us, adhoc_frame_fields_t *out_fields)
{
    if (sm == 0 || out_fields == 0 || sm->upstream_id == 0u || sm->joined_level == 0u) return 0;
    memset(out_fields, 0, sizeof(*out_fields));
    out_fields->msg_class  = ADHOC_MSG_CLASS_A;
    out_fields->gateway_no = sm->upstream_gateway_no;
    out_fields->slot_high4 = adhoc_sm_slot_match_level(slot_seed, sm->joined_level);
    out_fields->level      = sm->joined_level;
    out_fields->sender.domain_id = sm->cfg.domain_id;
    out_fields->sender.node_id   = sm->cfg.node_id;
    adhoc_sm_content_write_lmt_a(out_fields->content, now_us);
    if (!adhoc_sm_content_write_id(out_fields->content, 0u, ADHOC_PAYLOAD_ID_FLAG_UPSTREAM_CONFIRMED, sm->upstream_id)) return 0;
    if (!adhoc_reply_list_build_confirm_payload(&sm->downlink_confirm_list, out_fields->content,
                                                (uint8_t)(ADHOC_A_CONFIRM_PAYLOAD_OFFSET + ADHOC_REPLY_ITEM_ENCODED_LEN),
                                                ADHOC_REPLY_MAX_PER_FRAME - 1u, 0)) return 0;
    return 1;
}

/** @brief 组装网关广播 A V=0 帧: LMT_A + 确认列表(flag=7)×5 */
static int adhoc_sm_emit_gateway_beacon(adhoc_sm_t *sm, uint32_t now_us, adhoc_frame_fields_t *out_fields)
{
    if (sm == 0 || out_fields == 0) return 0;
    memset(out_fields, 0, sizeof(*out_fields));
    out_fields->msg_class  = ADHOC_MSG_CLASS_A;
    out_fields->gateway_no = sm->cfg.gateway_no;
    out_fields->slot_high4 = 0u;
    out_fields->level      = 0u;
    out_fields->sender.domain_id = sm->cfg.domain_id;
    out_fields->sender.node_id   = sm->cfg.node_id;
    adhoc_sm_content_write_lmt_a(out_fields->content, now_us);
    if (!adhoc_reply_list_build_confirm_payload(&sm->downlink_confirm_list, out_fields->content,
                                                ADHOC_A_CONFIRM_PAYLOAD_OFFSET, ADHOC_REPLY_MAX_PER_FRAME, 0))
        return 0;
    return 1;
}

/* ========== 状态迁移函数 ========== */

/** @brief 进入 ST1: 等待组网, 清理所有上级信息、邻居缓存、确认队列 */
static void adhoc_sm_enter_st1(adhoc_sm_t *sm)
{
    if (sm == 0) return;
    sm->state = ADHOC_SM_STATE_ST1;
    sm->joined_level = 0u;
    sm->upstream_id = sm->upstream_no = sm->upstream_gateway_no = 0u;
    sm->upstream_gateway_no = sm->cfg.gateway_no;
    sm->retry_count = 0u;
    sm->next_tx_us = sm->upstream_last_seen_us = 0u;
    sm->gateway_network_started = sm->gateway_network_locked = 0u;
    sm->gateway_network_start_us = sm->gateway_network_end_us = 0u;
    sm->gateway_rx_window_open = sm->gateway_rx_window_start_us = sm->gateway_rx_window_end_us = 0u;
    sm->network_lock_active = sm->network_lock_closed = 0u;
    sm->network_lock_end_us = 0u;
    sm->regroup_timer_active = sm->regroup_start_us = 0u;
    adhoc_sm_candidate_clear(&sm->candidate);
    adhoc_sm_neighbor_cache_reset(sm);
    adhoc_sm_upstream_bindings_reset(sm);
    adhoc_sm_downstream_bindings_reset(sm);
    adhoc_reply_list_reset(&sm->downlink_confirm_list);
}

/** @brief 进入未确认状态(U1/UN): 从候选邻居中提取上级信息, 清理其他候选 */
static void adhoc_sm_enter_un(adhoc_sm_t *sm, uint32_t ts_us, uint8_t joined_level)
{
    if (sm == 0 || sm->candidate.active == 0u || joined_level == 0u) return;
    sm->state = joined_level == 1u ? ADHOC_SM_STATE_U1 : ADHOC_SM_STATE_UN;
    sm->joined_level = joined_level;
    sm->upstream_id = sm->candidate.upstream_id;
    sm->upstream_no  = sm->candidate.upstream_no;
    sm->upstream_gateway_no = sm->candidate.gateway_no;
    sm->retry_count = 0u;
    sm->next_tx_us  = ts_us;
    sm->upstream_last_seen_us = ts_us;
    sm->network_lock_active = sm->candidate.network_end_us != 0u ? 1u : 0u;
    sm->network_lock_closed = 0u;
    sm->network_lock_end_us = sm->candidate.network_end_us;
    sm->regroup_timer_active = sm->regroup_start_us = 0u;
    adhoc_sm_neighbors_release_except(sm, sm->candidate.gateway_no, sm->candidate.upstream_level, sm->candidate.upstream_id);
    adhoc_sm_candidate_clear(&sm->candidate);
}

/** @brief 进入已确认状态(C1/CN): 根据 level 选择 C1 或 CN, 启动重组网定时器 */
static void adhoc_sm_enter_cn(adhoc_sm_t *sm, uint32_t ts_us)
{
    if (sm == 0 || sm->joined_level == 0u) return;
    sm->state = sm->joined_level == 1u ? ADHOC_SM_STATE_C1 : ADHOC_SM_STATE_CN;
    sm->retry_count = 0u;
    sm->next_tx_us = ts_us;
    sm->upstream_last_seen_us = ts_us;
    if (sm->network_lock_active != 0u && sm->network_lock_end_us != 0u && adhoc_sm_time_reached(ts_us, sm->network_lock_end_us))
        sm->network_lock_closed = 1u;
    if (sm->cfg.regroup_interval_us != 0u) {
        sm->regroup_timer_active = 1u;
        sm->regroup_start_us = ts_us;
    } else {
        sm->regroup_timer_active = sm->regroup_start_us = 0u;
    }
}

/* ========== 状态机对外 API ========== */

int adhoc_sm_init(adhoc_sm_t *sm, const adhoc_sm_cfg_t *cfg)
{
    uint8_t retry_max;
    uint32_t network_window_us;
    if (sm == 0 || cfg == 0) return 0;
    if (cfg->domain_id > ADHOC_SENDER_DOMAIN_MAX || cfg->node_id == 0u || cfg->node_id > ADHOC_SENDER_NODE_MAX) return 0;
    if (cfg->gateway_no > 7u || cfg->t2_us == 0u || cfg->t5_us == 0u || cfg->t2_us >= cfg->t5_us) return 0;
    if (cfg->role != ADHOC_SM_ROLE_BEACON && cfg->role != ADHOC_SM_ROLE_GATEWAY) return 0;
    retry_max = cfg->retry_max == 0u ? ADHOC_SM_DEFAULT_RETRY_MAX : cfg->retry_max;
    network_window_us = cfg->network_window_us == 0u ? ADHOC_SM_DEFAULT_NETWORK_WINDOW_US : cfg->network_window_us;
    memset(sm, 0, sizeof(*sm));
    sm->cfg = *cfg;
    sm->cfg.retry_max = retry_max;
    sm->cfg.network_window_us = network_window_us;
    sm->role = cfg->role;
    sm->inited = 1u;
    adhoc_sm_enter_st1(sm);
    return 1;
}

int adhoc_sm_set_role(adhoc_sm_t *sm, uint8_t role)
{
    if (sm == 0 || sm->inited == 0u) return 0;
    if (role != ADHOC_SM_ROLE_BEACON && role != ADHOC_SM_ROLE_GATEWAY) return 0;
    sm->role = role;
    sm->cfg.role = role;
    adhoc_sm_enter_st1(sm);
    return 1;
}

void adhoc_sm_reset(adhoc_sm_t *sm)
{
    if (sm == 0 || sm->inited == 0u) return;
    adhoc_sm_enter_st1(sm);
}

/**
 * @brief 接收 A 帧处理入口
 *
 * 网关: 检查 T2 窗口 → 匹配入网请求(flag=0~5) → 入确认队列
 * 信标:
 *   - 更新上级 last_seen
 *   - 检查下级入网请求 → 入确认队列
 *   - 未确认态检查 flag=7 确认 → 转入 C1/CN
 *   - ST1 态记录邻居 → 选择最优候选 → 转入 U1/UN
 */
adhoc_sm_rx_result_t adhoc_sm_on_rx(adhoc_sm_t *sm, const adhoc_sm_rx_event_t *event)
{
    uint8_t signal_rank, join_level;
    uint8_t child_bind_no;
    if (sm == 0 || event == 0 || sm->inited == 0u) return ADHOC_SM_RX_IGNORED;
    if (event->sender.domain_id != sm->cfg.domain_id) return ADHOC_SM_RX_IGNORED;

    /* === 网关处理 === */
    if (sm->role == ADHOC_SM_ROLE_GATEWAY) {
        if (!adhoc_sm_gateway_in_rx_window(sm, event->ts_us)) return ADHOC_SM_RX_IGNORED;
        if (event->msg_class != ADHOC_MSG_CLASS_A || event->level == 0u) return ADHOC_SM_RX_IGNORED;
        if (!adhoc_sm_sender_matches_level(event->level, event->sender.node_id)) return ADHOC_SM_RX_IGNORED;
        if (adhoc_sm_content_has_target_flag_range(event->content, sm->cfg.node_id,
                                                   ADHOC_PAYLOAD_ID_FLAG_BIND_MIN, ADHOC_PAYLOAD_ID_FLAG_BIND_MAX)) {
            if (!adhoc_reply_list_push_confirm_unique(&sm->downlink_confirm_list, event->sender.node_id))
                return ADHOC_SM_RX_IGNORED;
            return ADHOC_SM_RX_ACCEPTED;
        }
        if (adhoc_sm_content_has_target_flag_range(event->content, sm->cfg.node_id,
                                                   ADHOC_PAYLOAD_ID_FLAG_UPSTREAM_CONFIRMED,
                                                   ADHOC_PAYLOAD_ID_FLAG_UPSTREAM_CONFIRMED))
            return ADHOC_SM_RX_ACCEPTED;
        return ADHOC_SM_RX_IGNORED;
    }

    /* === 信标处理 === */

    /* 更新上级最近观测时间 */
    if (adhoc_sm_event_is_current_upstream(sm, event))
        sm->upstream_last_seen_us = event->ts_us;

    /* 已入网信标检查下级入网请求 */
    if (sm->joined_level != 0u && sm->network_lock_closed == 0u &&
        event->msg_class == ADHOC_MSG_CLASS_A &&
        event->level == (uint8_t)(sm->joined_level + 1u) &&
        event->gateway_no == sm->upstream_gateway_no &&
        adhoc_sm_sender_matches_level(event->level, event->sender.node_id)) {
        if (adhoc_sm_content_find_target_bind_no(event->content, sm->cfg.node_id, &child_bind_no)) {
            adhoc_sm_downstream_binding_upsert(sm, event->sender.node_id, child_bind_no,
                                               event->level, event->gateway_no, event->ts_us);
            if (!adhoc_reply_list_push_confirm_unique(&sm->downlink_confirm_list, event->sender.node_id))
                return ADHOC_SM_RX_IGNORED;
            return ADHOC_SM_RX_ACCEPTED;
        }
        if (adhoc_sm_content_has_target_flag_range(event->content, sm->cfg.node_id,
                                                   ADHOC_PAYLOAD_ID_FLAG_UPSTREAM_CONFIRMED,
                                                   ADHOC_PAYLOAD_ID_FLAG_UPSTREAM_CONFIRMED)) {
            adhoc_sm_downstream_binding_touch(sm, event->sender.node_id, event->level, event->gateway_no, event->ts_us);
            return ADHOC_SM_RX_ACCEPTED;
        }
    }

    /* 未确认态: 检查入网确认(flag=7 + 自身 node_id) */
    if (adhoc_sm_is_unconfirmed_state(sm->state) &&
        event->msg_class == ADHOC_MSG_CLASS_A &&
        event->gateway_no == sm->upstream_gateway_no &&
        event->sender.node_id == sm->upstream_id &&
        event->level + 1u == sm->joined_level &&
        adhoc_sm_content_has_confirm_self(event->content, sm->cfg.node_id)) {
        adhoc_sm_enter_cn(sm, event->ts_us);
        return ADHOC_SM_RX_STATE_CHANGED;
    }

    /* ST1 态: 扫描邻居, 评估入网条件 */
    if (sm->state != ADHOC_SM_STATE_ST1) return ADHOC_SM_RX_IGNORED;
    if (event->msg_class != ADHOC_MSG_CLASS_A || event->level >= ADHOC_SM_JOIN_LEVEL_MAX) return ADHOC_SM_RX_IGNORED;
    if (!adhoc_sm_sender_matches_level(event->level, event->sender.node_id)) return ADHOC_SM_RX_IGNORED;

    signal_rank = adhoc_sm_rssi_rank(event->rssi);
    if (!adhoc_sm_neighbor_record(sm, event, signal_rank)) return ADHOC_SM_RX_IGNORED;
    if (!adhoc_sm_select_best_ready_neighbor(sm, event->ts_us)) return ADHOC_SM_RX_ACCEPTED;

    join_level = (uint8_t)(sm->candidate.upstream_level + 1u);
    if (join_level == 0u || join_level > ADHOC_SM_JOIN_LEVEL_MAX) {
        adhoc_sm_candidate_clear(&sm->candidate);
        return ADHOC_SM_RX_IGNORED;
    }
    adhoc_sm_enter_un(sm, event->ts_us, join_level);
    return ADHOC_SM_RX_STATE_CHANGED;
}

/**
 * @brief 状态机周期轮询
 *
 * 网关: 启动组网 → 按 T5 周期发送 A V=0 → 开启 T2 收集窗口 → 时间窗到期锁定
 * 信标 ST1: 扫描过期邻居 → 选择最优就绪候选 → 转入 U1/UN
 * 信标 U1/UN: 检查超时/窗口到期 → 按 T5 发送入网请求(自增 retry) → 超过 retry_max 回退 ST1
 * 信标 C1/CN: 检查上级丢失/窗口关闭/重组网到期 → 按 T5 广播 A V=n
 */
int adhoc_sm_poll(adhoc_sm_t *sm, uint32_t now_us, adhoc_frame_fields_t *out_fields, uint8_t *out_has_tx)
{
    uint32_t cycle_idx, upstream_loss_timeout_us;
    uint8_t join_level;

    if (sm == 0 || out_has_tx == 0 || sm->inited == 0u) return 0;
    *out_has_tx = 0u;
    upstream_loss_timeout_us = adhoc_sm_upstream_loss_timeout_us(sm);

    /* === 网关轮询 === */
    if (sm->role == ADHOC_SM_ROLE_GATEWAY) {
        if (sm->gateway_network_started == 0u) {
            sm->gateway_network_started = 1u;
            sm->gateway_network_start_us = now_us;
            sm->gateway_network_end_us = now_us + sm->cfg.network_window_us;
        }
        if (sm->gateway_network_locked == 0u &&
            (uint32_t)(now_us - sm->gateway_network_start_us) >= sm->cfg.network_window_us) {
            sm->gateway_network_locked = 1u;
            sm->gateway_rx_window_open = 0u;
        }
        if (sm->gateway_rx_window_open != 0u &&
            (uint32_t)(now_us - sm->gateway_rx_window_start_us) > sm->cfg.t2_us)
            sm->gateway_rx_window_open = 0u;
        if (sm->gateway_network_locked != 0u) return 1;
        if (now_us < sm->next_tx_us) return 1;
        if (out_fields == 0 || !adhoc_sm_emit_gateway_beacon(sm, now_us, out_fields)) return 0;
        *out_has_tx = 1u;
        sm->gateway_rx_window_open = 1u;
        sm->gateway_rx_window_start_us = now_us;
        sm->gateway_rx_window_end_us = now_us + sm->cfg.t2_us;
        sm->next_tx_us = now_us + sm->cfg.t5_us;
        return 1;
    }

    /* === 信标 ST1: 等待组网 === */
    if (sm->state == ADHOC_SM_STATE_ST1) {
        cycle_idx = adhoc_sm_cycle_index(now_us, sm->cfg.t5_us);
        adhoc_sm_neighbors_sweep(sm, cycle_idx, now_us);
        if (adhoc_sm_select_best_ready_neighbor(sm, now_us)) {
            join_level = (uint8_t)(sm->candidate.upstream_level + 1u);
            if (join_level != 0u && join_level <= ADHOC_SM_JOIN_LEVEL_MAX) {
                adhoc_sm_enter_un(sm, now_us, join_level);
                return 1;
            }
            adhoc_sm_candidate_clear(&sm->candidate);
        }
    }

    /* === 信标 U1/UN: 未确认 === */
    if (adhoc_sm_is_unconfirmed_state(sm->state)) {
        if (sm->network_lock_closed == 0u && sm->upstream_last_seen_us != 0u &&
            upstream_loss_timeout_us != 0u &&
            (uint32_t)(now_us - sm->upstream_last_seen_us) > upstream_loss_timeout_us) {
            adhoc_sm_enter_st1(sm); return 1;
        }
        if (sm->network_lock_active != 0u && sm->network_lock_end_us != 0u &&
            adhoc_sm_time_reached(now_us, sm->network_lock_end_us)) {
            adhoc_sm_enter_st1(sm); return 1;
        }
        if (now_us < sm->next_tx_us) return 1;
        if (sm->retry_count >= sm->cfg.retry_max) { adhoc_sm_enter_st1(sm); return 1; }
        if (out_fields == 0 || !adhoc_sm_emit_join_response(sm, sm->retry_count, now_us, out_fields)) return 0;
        *out_has_tx = 1u;
        sm->retry_count++;
        sm->next_tx_us = now_us + sm->cfg.t5_us;
        return 1;
    }

    /* === 信标 C1/CN: 已确认 === */
    if (adhoc_sm_is_confirmed_state(sm->state)) {
        if (sm->network_lock_closed == 0u && sm->upstream_last_seen_us != 0u &&
            upstream_loss_timeout_us != 0u &&
            (uint32_t)(now_us - sm->upstream_last_seen_us) > upstream_loss_timeout_us) {
            adhoc_sm_enter_st1(sm); return 1;
        }
        if (sm->network_lock_active != 0u && sm->network_lock_end_us != 0u &&
            adhoc_sm_time_reached(now_us, sm->network_lock_end_us))
            sm->network_lock_closed = 1u;
        if (sm->regroup_timer_active != 0u && sm->cfg.regroup_interval_us != 0u &&
            (uint32_t)(now_us - sm->regroup_start_us) >= sm->cfg.regroup_interval_us) {
            adhoc_sm_enter_st1(sm); return 1;
        }
        if (sm->network_lock_closed != 0u) return 1;
        if (now_us < sm->next_tx_us) return 1;
        cycle_idx = adhoc_sm_cycle_index(now_us, sm->cfg.t5_us);
        if (out_fields == 0 || !adhoc_sm_emit_level_beacon(sm, (uint8_t)(cycle_idx & 0x0Fu), now_us, out_fields)) return 0;
        *out_has_tx = 1u;
        sm->next_tx_us = now_us + sm->cfg.t5_us;
        return 1;
    }

    return 1;
}

/** @brief 获取状态机只读快照 */
void adhoc_sm_get_snapshot(const adhoc_sm_t *sm, adhoc_sm_snapshot_t *out_snapshot)
{
    uint8_t index, count = 0u;
    if (sm == 0 || out_snapshot == 0) return;
    memset(out_snapshot, 0, sizeof(*out_snapshot));
    out_snapshot->state                    = sm->state;
    out_snapshot->joined_level             = sm->joined_level;
    out_snapshot->upstream_id              = sm->upstream_id;
    out_snapshot->upstream_no              = sm->upstream_no;
    out_snapshot->upstream_gateway_no      = sm->upstream_gateway_no;
    out_snapshot->retry_count              = sm->retry_count;
    out_snapshot->upstream_last_seen_us    = sm->upstream_last_seen_us;
    out_snapshot->gateway_network_started  = sm->gateway_network_started;
    out_snapshot->gateway_network_locked   = sm->gateway_network_locked;
    out_snapshot->gateway_network_start_us = sm->gateway_network_start_us;
    out_snapshot->gateway_network_end_us   = sm->gateway_network_end_us;
    out_snapshot->network_lock_active      = sm->network_lock_active;
    out_snapshot->network_lock_closed      = sm->network_lock_closed;
    out_snapshot->network_lock_end_us      = sm->network_lock_end_us;
    for (index = 0u; index < ADHOC_SM_DOWNSTREAM_BINDING_CAPACITY; ++index) {
        out_snapshot->downstream_bindings[index] = sm->downstream_bindings[index];
        if (sm->downstream_bindings[index].used != 0u) count++;
    }
    out_snapshot->downstream_binding_count = count;
}
