#ifndef ADHOC_API_H
#define ADHOC_API_H

#include "adhoc_link.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADHOC_FRAME_LEN 32u
#define ADHOC_DATA_USER_LEN 17u

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
    uint32_t lmt_d;
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
adhoc_rc_t adhoc_node_submit_data(void *node, uint8_t source_id_flag, uint32_t lmt_d,
                                  const uint8_t user[ADHOC_DATA_USER_LEN], uint32_t now_us);
adhoc_rc_t adhoc_node_fetch_data_tx_report(void *node, adhoc_node_data_tx_report_t *out_report);
adhoc_rc_t adhoc_node_get_runtime_status(void *node, adhoc_node_runtime_status_t *out_status);

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_API_H */
