> 注意：本文件是早期头文件摘录，尚未跟随 `T11` 时间戳模型迁移更新；其中 `user[18]`、`4` 条 ACK、`6` 条确认等常量仅代表历史版本。最新目标以 `docs/设计文档（初稿）.md`、`docs/任务分解技术书.md` 与实际 `.h` 文件为准。

#ifndef ADHOC_API_H
#define ADHOC_API_H

#include "adhoc_link.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADHOC_FRAME_LEN 32u
#define ADHOC_DATA_USER_LEN 18u

typedef enum
{
    ADHOC_ROLE_BEACON = 0,
    ADHOC_ROLE_GATEWAY = 1
} adhoc_role_t;

typedef enum
{
    ADHOC_OK = 0,
    ADHOC_EINVAL = -1,
    ADHOC_ESTATE = -2,
    ADHOC_EBUSY = -3,
    ADHOC_ENOFRAME = -4
} adhoc_rc_t;

typedef struct
{
    uint16_t domain_id;
    uint32_t node_id;
    uint8_t gateway_no;
    uint32_t t1_us;
    uint32_t t2_us;
    uint32_t t3_us;
    uint32_t t4_us;
    uint8_t retry_max;
    uint32_t network_window_us;
    uint32_t regroup_interval_us;
} adhoc_cfg_t;

typedef struct
{
    uint8_t bytes[ADHOC_FRAME_LEN];
    uint8_t len;
    int8_t rssi;
    uint32_t ts_us;
} adhoc_frame_t;

typedef enum
{
    ADHOC_NODE_DATA_TX_REPORT_NONE = 0u,
    ADHOC_NODE_DATA_TX_REPORT_ACKED = 1u,
    ADHOC_NODE_DATA_TX_REPORT_RETRY_EXHAUSTED = 2u
} adhoc_node_data_tx_report_code_t;

typedef struct
{
    uint8_t code;
    uint8_t source_id_flag;
    uint32_t source_node_id;
    uint16_t seq_no;
    uint8_t retry_count;
} adhoc_node_data_tx_report_t;

typedef struct
{
    uint8_t state;
    uint8_t joined_level;
    uint32_t upstream_id;
    uint8_t upstream_no;
    uint8_t upstream_gateway_no;
    uint8_t retry_count;
    uint32_t upstream_last_seen_us;
    uint8_t gateway_network_started;
    uint8_t gateway_network_locked;
    uint32_t gateway_network_start_us;
    uint32_t gateway_network_end_us;
    uint8_t network_lock_active;
    uint8_t network_lock_closed;
    uint32_t network_lock_end_us;
} adhoc_node_runtime_status_t;

uint32_t adhoc_node_required_size(void);
adhoc_rc_t adhoc_node_init(void *node_mem, uint32_t node_mem_size,
                           const adhoc_cfg_t *cfg,
                           const adhoc_link_ops_t *link_ops, void *link_ctx);
adhoc_rc_t adhoc_node_set_role(void *node, adhoc_role_t role);
adhoc_rc_t adhoc_node_reset(void *node);
adhoc_rc_t adhoc_node_on_rx(void *node, const adhoc_frame_t *rx);
adhoc_rc_t adhoc_node_poll(void *node, uint32_t now_us);
adhoc_rc_t adhoc_node_fetch_tx(void *node, adhoc_frame_t *tx);
adhoc_rc_t adhoc_node_submit_data(void *node, uint8_t source_id_flag, uint16_t seq_no,
                                  const uint8_t user[ADHOC_DATA_USER_LEN], uint32_t now_us);
adhoc_rc_t adhoc_node_fetch_data_tx_report(void *node, adhoc_node_data_tx_report_t *out_report);
adhoc_rc_t adhoc_node_get_runtime_status(void *node, adhoc_node_runtime_status_t *out_status);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_API_H */

---

#ifndef ADHOC_CRC8_H
#define ADHOC_CRC8_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADHOC_CRC8_POLY 0x07u
#define ADHOC_CRC8_INIT 0x00u
#define ADHOC_CRC8_XOROUT 0x00u

uint8_t adhoc_crc8_compute(const uint8_t *data, uint16_t len);
uint8_t adhoc_crc8_frame(const uint8_t frame[32]);
int adhoc_crc8_verify_frame(const uint8_t frame[32]);
void adhoc_crc8_write_frame(uint8_t frame[32]);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_CRC8_H */

---

#ifndef ADHOC_DATA_PLANE_H
#define ADHOC_DATA_PLANE_H

#include "adhoc_frame.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADHOC_DATA_MSG_USER_LEN 18u
#define ADHOC_DATA_ACK_MAX_PER_FRAME 4u
#define ADHOC_DATA_DEDUP_CAPACITY 64u
#define ADHOC_DATA_ACK_QUEUE_CAPACITY 32u
#define ADHOC_DATA_TX_QUEUE_CAPACITY 16u
#define ADHOC_DATA_TX_REPORT_QUEUE_CAPACITY 16u
#define ADHOC_DATA_TX_RETRY_MAX 30u
#define ADHOC_DATA_DEFAULT_DEDUP_WINDOW_MS (6u * 60u * 60u * 1000u)
#define ADHOC_DATA_GATEWAY_ID_MAX 1000000u

typedef enum
{
    ADHOC_DATA_ORIGIN_UNKNOWN = 0u,
    ADHOC_DATA_ORIGIN_SOURCE = 1u,
    ADHOC_DATA_ORIGIN_FORWARD = 2u
} adhoc_data_origin_t;

typedef enum
{
    ADHOC_DATA_RX_IGNORED = 0u,
    ADHOC_DATA_RX_ACCEPTED = 1u,
    ADHOC_DATA_RX_DUPLICATE = 2u,
    ADHOC_DATA_RX_ACK = 3u
} adhoc_data_rx_result_t;

typedef struct
{
    adhoc_payload_id_t source;
    uint16_t seq_no;
    uint8_t user[ADHOC_DATA_MSG_USER_LEN];
} adhoc_data_msg_t;

typedef struct
{
    adhoc_payload_id_t source;
    uint16_t seq_no;
} adhoc_data_ack_item_t;

typedef enum
{
    ADHOC_DATA_TX_REPORT_NONE = 0u,
    ADHOC_DATA_TX_REPORT_ACKED = 1u,
    ADHOC_DATA_TX_REPORT_RETRY_EXHAUSTED = 2u
} adhoc_data_tx_report_code_t;

typedef struct
{
    uint8_t code;
    adhoc_data_ack_item_t item;
    uint8_t retry_count;
} adhoc_data_tx_report_t;

typedef struct
{
    uint16_t domain_id;
    uint32_t node_id;
    uint8_t gateway_no;
    uint8_t role_gateway;
    uint32_t t5_us;
    uint32_t dedup_window_ms;
    uint8_t tx_retry_max;
} adhoc_data_plane_cfg_t;

typedef struct
{
    uint8_t used;
    adhoc_payload_id_t source;
    uint16_t seq_no;
    uint32_t last_seen_ms;
} adhoc_data_dedup_entry_t;

typedef struct
{
    uint8_t used;
    adhoc_data_ack_item_t item;
} adhoc_data_ack_queue_entry_t;

typedef struct
{
    uint8_t used;
    adhoc_data_msg_t msg;
    uint8_t level;
    uint8_t gateway_no;
    uint8_t retry_count;
    uint32_t next_tx_us;
} adhoc_data_tx_entry_t;

typedef struct
{
    uint8_t used;
    adhoc_data_tx_report_t report;
} adhoc_data_tx_report_entry_t;

typedef struct
{
    uint8_t inited;
    adhoc_data_plane_cfg_t cfg;
    uint8_t role_gateway;
    adhoc_data_dedup_entry_t dedup[ADHOC_DATA_DEDUP_CAPACITY];
    adhoc_data_ack_queue_entry_t ack_queue[ADHOC_DATA_ACK_QUEUE_CAPACITY];
    uint8_t ack_head;
    uint8_t ack_tail;
    uint8_t ack_count;
    uint32_t next_ack_tx_us;
    adhoc_data_tx_entry_t tx_queue[ADHOC_DATA_TX_QUEUE_CAPACITY];
    adhoc_data_tx_report_entry_t tx_report_queue[ADHOC_DATA_TX_REPORT_QUEUE_CAPACITY];
    uint8_t tx_report_head;
    uint8_t tx_report_tail;
    uint8_t tx_report_count;
    uint8_t joined_level;
    uint8_t upstream_no;
    uint8_t upstream_gateway_no;
    uint8_t has_last_rx_data;
    adhoc_data_msg_t last_rx_data;
    adhoc_data_origin_t last_rx_origin;
    uint8_t last_rx_duplicate;
    uint8_t last_ack_hit_count;
    uint8_t last_rx_ack_count;
    adhoc_data_ack_item_t last_rx_acks[ADHOC_DATA_ACK_MAX_PER_FRAME];
} adhoc_data_plane_t;

int adhoc_data_msg_pack(const adhoc_data_msg_t *msg, uint8_t payload_out[ADHOC_FRAME_PAYLOAD_LEN]);
int adhoc_data_msg_unpack(const uint8_t payload[ADHOC_FRAME_PAYLOAD_LEN], adhoc_data_msg_t *msg_out);
int adhoc_data_ack_payload_pack(const adhoc_data_ack_item_t *items, uint8_t item_count,
                                uint8_t payload_out[ADHOC_FRAME_PAYLOAD_LEN]);
int adhoc_data_ack_payload_unpack(const uint8_t payload[ADHOC_FRAME_PAYLOAD_LEN],
                                  adhoc_data_ack_item_t items_out[ADHOC_DATA_ACK_MAX_PER_FRAME],
                                  uint8_t *item_count_out);

int adhoc_data_plane_init(adhoc_data_plane_t *plane, const adhoc_data_plane_cfg_t *cfg);
int adhoc_data_plane_set_role(adhoc_data_plane_t *plane, uint8_t role_gateway);
int adhoc_data_plane_set_route(adhoc_data_plane_t *plane, uint8_t joined_level, uint8_t upstream_gateway_no, uint8_t upstream_no);
void adhoc_data_plane_reset(adhoc_data_plane_t *plane);
int adhoc_data_plane_submit_source_data(adhoc_data_plane_t *plane, uint8_t source_id_flag, uint16_t seq_no,
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

---
#ifndef ADHOC_FRAME_H
#define ADHOC_FRAME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADHOC_FRAME_SIZE 32u
#define ADHOC_FRAME_IDX_HEAD 0u
#define ADHOC_FRAME_IDX_LEVEL 1u
#define ADHOC_FRAME_IDX_SENDER 2u
#define ADHOC_FRAME_SENDER_LEN 5u
#define ADHOC_FRAME_IDX_PAYLOAD 7u
#define ADHOC_FRAME_PAYLOAD_LEN 24u
#define ADHOC_FRAME_IDX_CRC 31u

#define ADHOC_SENDER_DOMAIN_MAX 0x07FFu
#define ADHOC_SENDER_NODE_MAX 0x1FFFFFFFu
#define ADHOC_PAYLOAD_ID_FLAG_MAX 0x07u

typedef enum
{
    ADHOC_MSG_CLASS_A = 0u,
    ADHOC_MSG_CLASS_D = 1u
} adhoc_msg_class_t;

typedef struct
{
    uint16_t domain_id;
    uint32_t node_id;
} adhoc_sender_t;

typedef struct
{
    uint8_t id_flag;
    uint32_t node_id;
} adhoc_payload_id_t;

typedef struct
{
    uint8_t msg_class;
    uint8_t gateway_no;
    uint8_t slot_high4;
    uint8_t level;
    adhoc_sender_t sender;
    uint8_t payload[ADHOC_FRAME_PAYLOAD_LEN];
    uint8_t crc8;
} adhoc_frame_fields_t;

uint8_t adhoc_frame_header_make(uint8_t msg_class, uint8_t gateway_no, uint8_t slot_high4);
void adhoc_frame_header_parse(uint8_t header_byte, uint8_t *msg_class, uint8_t *gateway_no, uint8_t *slot_high4);
int adhoc_sender_pack(adhoc_sender_t sender, uint8_t out[ADHOC_FRAME_SENDER_LEN]);
int adhoc_sender_unpack(const uint8_t in[ADHOC_FRAME_SENDER_LEN], adhoc_sender_t *sender);
int adhoc_payload_id_pack(adhoc_payload_id_t packed_id, uint8_t out[4]);
int adhoc_payload_id_unpack(const uint8_t in[4], adhoc_payload_id_t *packed_id);
int adhoc_frame_build(const adhoc_frame_fields_t *fields, uint8_t out_frame[ADHOC_FRAME_SIZE]);
int adhoc_frame_parse(const uint8_t frame[ADHOC_FRAME_SIZE], adhoc_frame_fields_t *out_fields);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_FRAME_H */

---

#ifndef ADHOC_LINK_H
#define ADHOC_LINK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int (*init)(void *ctx);
    int (*start_rx)(void *ctx);
    int (*tx)(void *ctx, const uint8_t *buf, uint16_t len);
    int (*poll_rx)(void *ctx, uint8_t *buf, uint16_t *len, int8_t *rssi);
    uint32_t (*now_us)(void *ctx);
    uint16_t (*rand_u16)(void *ctx);
} adhoc_link_ops_t;

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_LINK_H */

---

#ifndef ADHOC_REPLY_LIST_H
#define ADHOC_REPLY_LIST_H

#include "adhoc_frame.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADHOC_REPLY_LIST_CAPACITY 24u
#define ADHOC_REPLY_MAX_PER_FRAME 6u
#define ADHOC_REPLY_ITEM_ENCODED_LEN 4u
#define ADHOC_A_CONFIRM_PAYLOAD_BYTES (ADHOC_REPLY_MAX_PER_FRAME * ADHOC_REPLY_ITEM_ENCODED_LEN)
#define ADHOC_A_JOIN_HEAD_BYTES ADHOC_REPLY_ITEM_ENCODED_LEN

#if ADHOC_A_CONFIRM_PAYLOAD_BYTES != ADHOC_FRAME_PAYLOAD_LEN
#error "Current A-frame protocol assumes 6 confirm IDs fully occupy the 24-byte payload."
#endif

#if (ADHOC_A_JOIN_HEAD_BYTES + ((ADHOC_REPLY_MAX_PER_FRAME - 1u) * ADHOC_REPLY_ITEM_ENCODED_LEN)) != ADHOC_FRAME_PAYLOAD_LEN
#error "Current join-response protocol assumes 1 upstream ID plus 5 confirm IDs fully occupy the 24-byte payload."
#endif

typedef struct
{
    adhoc_payload_id_t ids[ADHOC_REPLY_LIST_CAPACITY];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} adhoc_reply_list_t;

void adhoc_reply_list_reset(adhoc_reply_list_t *list);
int adhoc_reply_list_push_unique(adhoc_reply_list_t *list, adhoc_payload_id_t id);
int adhoc_reply_list_pop(adhoc_reply_list_t *list, adhoc_payload_id_t *out_id);
uint8_t adhoc_reply_list_size(const adhoc_reply_list_t *list);
int adhoc_reply_list_build_confirm_payload(adhoc_reply_list_t *list, uint8_t payload[ADHOC_FRAME_PAYLOAD_LEN],
                                           uint8_t max_ids, uint8_t *out_used_ids);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_REPLY_LIST_H */

---

#ifndef ADHOC_SM_H
#define ADHOC_SM_H

#include "adhoc_frame.h"
#include "adhoc_reply_list.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADHOC_SM_GATEWAY_ID_MAX 1000000u
#define ADHOC_SM_BEACON_ID_MIN 1000001u
#define ADHOC_SM_DEFAULT_RETRY_MAX 30u
#define ADHOC_SM_DEFAULT_NETWORK_WINDOW_US 30000000u
#define ADHOC_SM_RSSI_STRONG_MIN_DBM (-65)
#define ADHOC_SM_RSSI_MEDIUM_MIN_DBM (-80)
#define ADHOC_SM_JOIN_LEVEL_MAX 254u
#define ADHOC_SM_UPSTREAM_BINDING_CAPACITY 6u
#define ADHOC_SM_NEIGHBOR_CACHE_CAPACITY 6u
#define ADHOC_SM_UPSTREAM_LOSS_CYCLES 3u

typedef enum
{
    ADHOC_SM_STATE_ST0 = 0u,
    ADHOC_SM_STATE_ST1 = 1u,
    ADHOC_SM_STATE_U1 = 2u,
    ADHOC_SM_STATE_C1 = 3u,
    ADHOC_SM_STATE_UN = 4u,
    ADHOC_SM_STATE_CN = 5u
} adhoc_sm_state_t;

typedef enum
{
    ADHOC_SM_RX_IGNORED = 0,
    ADHOC_SM_RX_ACCEPTED = 1,
    ADHOC_SM_RX_STATE_CHANGED = 2
} adhoc_sm_rx_result_t;

typedef enum
{
    ADHOC_SM_ROLE_BEACON = 0u,
    ADHOC_SM_ROLE_GATEWAY = 1u
} adhoc_sm_role_t;

typedef struct
{
    uint8_t role;
    uint16_t domain_id;
    uint32_t node_id;
    uint8_t gateway_no;
    uint32_t t2_us;
    uint32_t t5_us;
    uint8_t retry_max;
    uint32_t network_window_us;
    uint32_t regroup_interval_us;
} adhoc_sm_cfg_t;

typedef struct
{
    uint8_t msg_class;
    uint8_t gateway_no;
    uint8_t level;
    adhoc_sender_t sender;
    uint8_t payload[ADHOC_FRAME_PAYLOAD_LEN];
    int8_t rssi;
    uint32_t ts_us;
} adhoc_sm_rx_event_t;

typedef struct
{
    adhoc_sm_state_t state;
    uint8_t joined_level;
    uint32_t upstream_id;
    uint8_t upstream_no;
    uint8_t upstream_gateway_no;
    uint8_t retry_count;
    uint32_t upstream_last_seen_us;
    uint8_t gateway_network_started;
    uint8_t gateway_network_locked;
    uint32_t gateway_network_start_us;
    uint32_t gateway_network_end_us;
    uint8_t network_lock_active;
    uint8_t network_lock_closed;
    uint32_t network_lock_end_us;
} adhoc_sm_snapshot_t;

typedef struct
{
    uint8_t active;
    uint8_t upstream_level;
    uint8_t upstream_no;
    uint8_t gateway_no;
    uint32_t upstream_id;
    uint8_t signal_rank;
    uint8_t required_hits;
    uint8_t window_cycles;
    uint8_t success_hits;
    int8_t last_rssi;
    uint32_t first_cycle_idx;
    uint32_t last_hit_cycle_idx;
    uint32_t last_seen_us;
    uint32_t network_end_us;
    uint8_t has_last_hit_cycle;
} adhoc_sm_candidate_t;

typedef struct
{
    uint8_t used;
    uint8_t upstream_no;
    uint8_t gateway_no;
    uint8_t upstream_level;
    uint32_t upstream_id;
} adhoc_sm_upstream_binding_t;

typedef struct
{
    uint8_t inited;
    adhoc_sm_cfg_t cfg;
    uint8_t role;
    adhoc_sm_state_t state;
    uint8_t joined_level;
    uint32_t upstream_id;
    uint8_t upstream_no;
    uint8_t upstream_gateway_no;
    uint8_t retry_count;
    uint32_t next_tx_us;
    uint32_t upstream_last_seen_us;
    uint8_t gateway_network_started;
    uint8_t gateway_network_locked;
    uint32_t gateway_network_start_us;
    uint32_t gateway_network_end_us;
    uint8_t gateway_rx_window_open;
    uint32_t gateway_rx_window_start_us;
    uint32_t gateway_rx_window_end_us;
    uint8_t network_lock_active;
    uint8_t network_lock_closed;
    uint32_t network_lock_end_us;
    uint8_t regroup_timer_active;
    uint32_t regroup_start_us;
    adhoc_sm_candidate_t candidate;
    adhoc_sm_candidate_t neighbor_cache[ADHOC_SM_NEIGHBOR_CACHE_CAPACITY];
    adhoc_sm_upstream_binding_t upstream_bindings[ADHOC_SM_UPSTREAM_BINDING_CAPACITY];
    adhoc_reply_list_t downlink_confirm_list;
} adhoc_sm_t;

int adhoc_sm_init(adhoc_sm_t *sm, const adhoc_sm_cfg_t *cfg);
int adhoc_sm_set_role(adhoc_sm_t *sm, uint8_t role);
void adhoc_sm_reset(adhoc_sm_t *sm);
adhoc_sm_rx_result_t adhoc_sm_on_rx(adhoc_sm_t *sm, const adhoc_sm_rx_event_t *event);
int adhoc_sm_poll(adhoc_sm_t *sm, uint32_t now_us, adhoc_frame_fields_t *out_fields, uint8_t *out_has_tx);
void adhoc_sm_get_snapshot(const adhoc_sm_t *sm, adhoc_sm_snapshot_t *out_snapshot);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_SM_H */

---

#ifndef ADHOC_TIMING_H
#define ADHOC_TIMING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADHOC_TMOS_TICK_US 625u
#define ADHOC_TIMING_M_RECOMMENDED_MIN 10u
#define ADHOC_TIMING_M_RECOMMENDED_MAX 1000u

typedef struct
{
    uint32_t t1_us;
    uint32_t t2_us;
    uint32_t t3_us;
    uint32_t t4_us;
} adhoc_timing_cfg_t;

typedef struct
{
    uint32_t t1_us;
    uint32_t t2_us;
    uint32_t t3_us;
    uint32_t t4_us;
    uint32_t t5_us;
    uint32_t t6_us;
    uint16_t n_slots;
    uint16_t m_wait;
} adhoc_timing_plan_t;

int adhoc_timing_build(const adhoc_timing_cfg_t *cfg, adhoc_timing_plan_t *out_plan);
int adhoc_timing_slot_parity_match(uint8_t level, uint16_t slot_no);
int adhoc_timing_is_m_recommended(uint16_t m_wait);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_TIMING_H */
