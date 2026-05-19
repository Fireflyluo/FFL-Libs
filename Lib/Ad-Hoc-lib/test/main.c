/*-----------------------------------------------File Info------------------------------------------------
** File Name:               main.c
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            Ad-Hoc-lib 多节点仿真主程序
**                          在 Windows 上用多线程模拟嵌入式自组网协议运行,
**                          通过拓扑矩阵控制节点间可达性和 RSSI, 验证协议实现。
**
** 架构: 主线程定时打印全网状态 / N 个节点线程各自运行 poll→fetch_tx→on_rx 循环
** 场景切换: 修改 SCENARIO_MODE 宏 (0~6)
** 编译: gcc -std=gnu11 或 MSVC cl
**--------------------------------------------------------------------------------------------------------
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>

#include "adhoc_api.h"
#include "adhoc_sm.h"

#include "channel_win32/adhoc_channel.h"
#include "channel_win32/adhoc_link_win32.h"
#include "channel_win32/adhoc_logger.h"
#include "channel_win32/adhoc_virtual_time.h"

/** @brief 仿真最大节点数 */
#define SIM_MAX_NODES       16u
/** @brief 节点线程轮询间隔(ms) */
#define SIM_POLL_INTERVAL_MS 1u

/** @brief 场景模式选择: 0全连通 1树形网状 2弱信号 3高频 4重组网 5压力混合 6长时间 7转发延迟 */
#define SCENARIO_MODE        0u

#if SCENARIO_MODE == 0u
#define SCENARIO_GW_COUNT    1u
#define SCENARIO_BCN_COUNT   2u
#define SCENARIO_RSSI        -40
#define SIM_RUN_DURATION_S   35u
#define SIM_DATA_INTERVAL_US 2000000u
#define SIM_REGROUP_US       0u
#elif SCENARIO_MODE == 1u
#define SCENARIO_GW_COUNT    2u
#define SCENARIO_BCN_COUNT   7u
#define SCENARIO_RSSI        -40
#define SIM_RUN_DURATION_S   35u
#define SIM_DATA_INTERVAL_US 2000000u
#define SIM_REGROUP_US       0u
#elif SCENARIO_MODE == 2u
#define SCENARIO_GW_COUNT    1u
#define SCENARIO_BCN_COUNT   2u
#define SCENARIO_RSSI        -90
#define SIM_RUN_DURATION_S   20u
#define SIM_DATA_INTERVAL_US 2000000u
#define SIM_REGROUP_US       0u
#elif SCENARIO_MODE == 3u
#define SCENARIO_GW_COUNT    1u
#define SCENARIO_BCN_COUNT   2u
#define SCENARIO_RSSI        -40
#define SIM_RUN_DURATION_S   20u
#define SIM_DATA_INTERVAL_US 60000u
#define SIM_REGROUP_US       0u
#elif SCENARIO_MODE == 4u
#define SCENARIO_GW_COUNT    1u
#define SCENARIO_BCN_COUNT   2u
#define SCENARIO_RSSI        -40
#define SIM_RUN_DURATION_S   70u
#define SIM_DATA_INTERVAL_US 2000000u
#define SIM_REGROUP_US       60000000u
#elif SCENARIO_MODE == 5u
#define SCENARIO_GW_COUNT    1u
#define SCENARIO_BCN_COUNT   15u
#define SCENARIO_RSSI        -40
#define SIM_RUN_DURATION_S   30u
#define SIM_DATA_INTERVAL_US 60000u
#define SIM_REGROUP_US       0u
#elif SCENARIO_MODE == 6u
#define SCENARIO_GW_COUNT    1u
#define SCENARIO_BCN_COUNT   2u
#define SCENARIO_RSSI        -40
#define SIM_RUN_DURATION_S   600u
#define SIM_DATA_INTERVAL_US 2000000u
#define SIM_REGROUP_US       0u
#elif SCENARIO_MODE == 7u
#define SCENARIO_GW_COUNT    1u
#define SCENARIO_BCN_COUNT   4u
#define SCENARIO_RSSI        -40
#define SIM_RUN_DURATION_S   40u
#define SIM_DATA_INTERVAL_US 1000000u
#define SIM_REGROUP_US       0u
#endif

#define SCENARIO_NODES       (SCENARIO_GW_COUNT + SCENARIO_BCN_COUNT)

static const adhoc_cfg_t g_default_cfg = {
    1u, 0u, 0u,
    2500u, 57500u, 62500u, 1937500u,
    30u, 30000000u, SIM_REGROUP_US
};

/**
 * @brief 仿真节点实例(含协议栈内存+链路上下文+线程句柄)
 */
typedef struct
{
    uint32_t       node_id;           /**< 协议节点ID */
    uint8_t        role;              /**< 角色(GATEWAY/BEACON) */
    uint8_t        gateway_no;        /**< 网关编号 */
    uint8_t        node_idx;          /**< 在信道中的索引 */
    uint8_t        log_slot;          /**< 日志槽索引 */
    uint8_t        node_mem[8192u];   /**< 协议栈静态内存 */
    adhoc_link_win32_ctx_t link_ctx;  /**< 链路适配上下文 */
    HANDLE         thread;            /**< Win32 线程句柄 */
    volatile LONG  running;           /**< 线程运行标志 */
    volatile LONG  stop;              /**< 线程停止标志 */
    uint32_t       last_state;        /**< 上次状态(用于检测变化) */
    uint32_t       last_data_submit_us; /**< 上次数据提交时刻 */
    uint32_t       data_seq;          /**< 数据序号 */
    uint32_t       submit_time_us[256]; /**< 提交时间表(按seq%256索引) */
    uint8_t        network_locked_logged; /**< 是否已打印 NETWORK_LOCKED */
} sim_node_t;

/**
 * @brief 仿真全局上下文
 */
typedef struct
{
    adhoc_virtual_time_t vt;          /**< 虚拟时间(全局共享) */
    adhoc_channel_t      channel;     /**< 空域信道(全局共享) */
    sim_node_t           nodes[SIM_MAX_NODES]; /**< 节点数组 */
    uint8_t              node_count;  /**< 当前节点数 */
    uint8_t              gw_count;    /**< 网关数 */
    uint8_t              bcn_count;   /**< 信标数 */
    int8_t               topo[SIM_MAX_NODES][SIM_MAX_NODES]; /**< 拓扑矩阵副本 */
    volatile LONG        stop;        /**< 全局停止标志 */
} sim_ctx_t;

static sim_ctx_t g_sim;

/** @brief 状态枚举 → 可读字符串 */
static const char *state_name(uint8_t state)
{
    switch (state)
    {
    case ADHOC_SM_STATE_ST0: return "ST0";
    case ADHOC_SM_STATE_ST1: return "ST1";
    case ADHOC_SM_STATE_U1:  return "U1";
    case ADHOC_SM_STATE_C1:  return "C1";
    case ADHOC_SM_STATE_UN:  return "UN";
    case ADHOC_SM_STATE_CN:  return "CN";
    default:                 return "??";
    }
}

/**
 * @brief 节点线程主循环
 *
 * 每 1ms 执行: poll → fetch_tx → channel_tx / poll_rx → on_rx → fetch_report
 * 监控状态变化并记录日志, 已入网信标定时提交业务数据
 */
static DWORD WINAPI node_thread(LPVOID param)
{
    sim_node_t             *node = (sim_node_t *)param;
    adhoc_frame_t           tx_frame;
    adhoc_node_data_tx_report_t report;
    adhoc_node_runtime_status_t status;
    adhoc_frame_t           rx_frame;
    uint8_t                 rx_buf[ADHOC_FRAME_LEN];
    adhoc_rc_t              rc;
    uint32_t                now_us;
    uint8_t                 prev_state = 0xFFu;
    uint8_t                 prev_joined = 0xFFu;
    uint8_t                 data_user[ADHOC_DATA_USER_LEN];
    memset(data_user, 0xABu, sizeof(data_user));

    while (!InterlockedCompareExchange(&node->stop, 0L, 0L))
    {
        now_us = adhoc_virtual_time_now_us(&g_sim.vt);

        rc = adhoc_node_poll(node->node_mem, now_us);
        if (rc != ADHOC_OK && rc != ADHOC_EBUSY)
        {
            adhoc_logger_log(node->log_slot, "[NODE:%u] poll err=%d", node->node_id, rc);
        }

        memset(&tx_frame, 0, sizeof(tx_frame));
        rc = adhoc_node_fetch_tx(node->node_mem, &tx_frame);
        if (rc == ADHOC_OK)
        {
            g_adhoc_link_win32_ops.tx(&node->link_ctx, tx_frame.bytes, tx_frame.len);
        }

        {
            uint16_t prx_len = 0u;
            int8_t   prx_rssi = 0;
            while (g_adhoc_link_win32_ops.poll_rx(&node->link_ctx, rx_buf, &prx_len, &prx_rssi) == ADHOC_LINK_OK)
            {
                memset(&rx_frame, 0, sizeof(rx_frame));
                memcpy(rx_frame.bytes, rx_buf, prx_len);
                rx_frame.len = (uint8_t)prx_len;
                rx_frame.rssi = prx_rssi;
                rx_frame.ts_us = now_us;
                adhoc_node_on_rx(node->node_mem, &rx_frame);
            }
        }

        while (adhoc_node_fetch_data_tx_report(node->node_mem, &report) == ADHOC_OK)
        {
            if (report.code == ADHOC_NODE_DATA_TX_REPORT_ACKED)
            {
                if (report.source_node_id == node->node_id)
                {
                    uint32_t lat_us = now_us - node->submit_time_us[report.lmt_d & 0xFFu];
                    adhoc_logger_log(node->log_slot,
                        "[NODE:%u] DATA ACKED: src=%u lmt_d=%u retry=%u latency=%ums",
                        node->node_id, report.source_node_id, report.lmt_d, report.retry_count, lat_us / 1000u);
                }
                else
                {
                    adhoc_logger_log(node->log_slot,
                        "[NODE:%u] DATA ACKED: src=%u lmt_d=%u retry=%u",
                        node->node_id, report.source_node_id, report.lmt_d, report.retry_count);
                }
            }
            else if (report.code == ADHOC_NODE_DATA_TX_REPORT_RETRY_EXHAUSTED)
            {
                adhoc_logger_log(node->log_slot,
                    "[NODE:%u] DATA RETRY_EXHAUSTED: src=%u lmt_d=%u retry=%u",
                    node->node_id, report.source_node_id, report.lmt_d, report.retry_count);
            }
        }

        memset(&status, 0, sizeof(status));
        if (adhoc_node_get_runtime_status(node->node_mem, &status) == ADHOC_OK)
        {
            if (status.state != prev_state)
            {
                prev_state = status.state;
                adhoc_logger_log(node->log_slot,
                    "[NODE:%u] STATE: %s  level=%u  upstream=%u  retry=%u",
                    node->node_id, state_name(status.state),
                    status.joined_level, status.upstream_id, status.retry_count);
            }
            if (status.joined_level != prev_joined)
            {
                prev_joined = status.joined_level;
                if (status.joined_level != 0u)
                {
                    adhoc_logger_log(node->log_slot,
                        "[NODE:%u] JOINED: level=%u upstream=%u upstream_no=%u gw_no=%u",
                        node->node_id, status.joined_level, status.upstream_id,
                        status.upstream_no, status.upstream_gateway_no);
                }
            }

            if (status.joined_level != 0u && node->role == ADHOC_ROLE_BEACON)
            {
                if ((uint32_t)(now_us - node->last_data_submit_us) >= SIM_DATA_INTERVAL_US)
                {
                    node->last_data_submit_us = now_us;
                    node->data_seq++;
                    data_user[0] = (uint8_t)(node->data_seq & 0xFFu);
                    data_user[1] = (uint8_t)((node->data_seq >> 8) & 0xFFu);

                    rc = adhoc_node_submit_data(node->node_mem,
                                                0u, node->data_seq,
                                                data_user, now_us);
                    if (rc == ADHOC_OK)
                    {
                        node->submit_time_us[node->data_seq & 0xFFu] = now_us;
                        adhoc_logger_log(node->log_slot,
                            "[NODE:%u] DATA SUBMIT: seq=%u",
                            node->node_id, node->data_seq);
                    }
                }
            }
        }

        if (status.gateway_network_locked != 0u && node->network_locked_logged == 0u)
        {
            node->network_locked_logged = 1u;
            adhoc_logger_log(node->log_slot,
                "[NODE:%u] NETWORK_LOCKED", node->node_id);
        }

        Sleep(SIM_POLL_INTERVAL_MS);
    }

    InterlockedExchange(&node->running, 0L);
    return 0;
}

/**
 * @brief 仿真场景初始化: 日志→信道→拓扑→N个节点(init+set_role)
 */
static void sim_init_scenario(void)
{
    uint8_t  i, j;
    adhoc_cfg_t cfg;
    char     log_name[64];

    memset(&g_sim, 0, sizeof(g_sim));
    g_sim.gw_count  = SCENARIO_GW_COUNT;
    g_sim.bcn_count = SCENARIO_BCN_COUNT;
    g_sim.node_count = SCENARIO_NODES;

    adhoc_virtual_time_init(&g_sim.vt);

    {
        char exe_dir[512];
        char logs_path[512];
        DWORD len = GetModuleFileNameA(NULL, exe_dir, sizeof(exe_dir));
        while (len > 0u && exe_dir[len - 1u] != '\\' && exe_dir[len - 1u] != '/') { len--; }
        exe_dir[len] = '\0';
        snprintf(logs_path, sizeof(logs_path), "%s\\logs", exe_dir);
        adhoc_logger_init(logs_path);
    }

    adhoc_channel_init(&g_sim.channel, g_sim.node_count);

    {
        int8_t (*topo)[SIM_MAX_NODES] = g_sim.topo;
        memset(topo, 0, sizeof(g_sim.topo));

#if SCENARIO_MODE == 1u
        topo[0][2] = -40; topo[2][0] = -40;
        topo[0][3] = -40; topo[3][0] = -40;
        topo[0][4] = -70; topo[4][0] = -70;
        topo[1][4] = -40; topo[4][1] = -40;
        topo[1][2] = -70; topo[2][1] = -70;
        topo[2][5] = -50; topo[5][2] = -50;
        topo[3][6] = -50; topo[6][3] = -50;
        topo[4][7] = -50; topo[7][4] = -50;
        topo[5][8] = -50; topo[8][5] = -50;
        topo[6][7] = -50; topo[7][6] = -50;
#elif SCENARIO_MODE == 7u
        topo[0][1] = -40; topo[1][0] = -40;
        topo[1][2] = -50; topo[2][1] = -50;
        topo[2][3] = -50; topo[3][2] = -50;
        topo[3][4] = -50; topo[4][3] = -50;
#else
        for (i = 0u; i < g_sim.node_count; ++i)
            for (j = 0u; j < g_sim.node_count; ++j)
                if (i != j) topo[i][j] = SCENARIO_RSSI;
#endif

        for (i = 0u; i < g_sim.node_count; ++i)
            for (j = 0u; j < g_sim.node_count; ++j)
                if (topo[i][j] != 0)
                    adhoc_channel_set_topo(&g_sim.channel, i, j, topo[i][j]);
    }

    for (i = 0u; i < g_sim.node_count; ++i)
    {
        sim_node_t *node = &g_sim.nodes[i];
        memset(node, 0, sizeof(*node));

        if (i < g_sim.gw_count)
        {
            node->node_id    = 1u + (uint32_t)i;
            node->role       = ADHOC_ROLE_GATEWAY;
            node->gateway_no = i;
        }
        else
        {
            node->node_id    = 1000001u + (uint32_t)(i - g_sim.gw_count);
            node->role       = ADHOC_ROLE_BEACON;
            node->gateway_no = 0u;
        }

        node->node_idx   = i;
        node->log_slot   = i;
        node->data_seq   = 0u;

        snprintf(log_name, sizeof(log_name), "node_%u_%u", i, node->node_id);
        adhoc_logger_open_slot(i, log_name);

        adhoc_link_win32_ctx_init(&node->link_ctx,
                                  &g_sim.channel, i, i,
                                  &g_sim.vt, node->node_id);

        cfg = g_default_cfg;
        cfg.domain_id  = 1u;
        cfg.node_id    = node->node_id;
        cfg.gateway_no = node->gateway_no;

        adhoc_rc_t rc = adhoc_node_init(node->node_mem, sizeof(node->node_mem),
                                        &cfg, &g_adhoc_link_win32_ops, &node->link_ctx);
        if (rc != ADHOC_OK)
        {
            printf("[FATAL] node %u init failed: %d\n", node->node_id, rc);
            exit(1);
        }

        rc = adhoc_node_set_role(node->node_mem, node->role);
        if (rc != ADHOC_OK)
        {
            printf("[FATAL] node %u set_role failed: %d\n", node->node_id, rc);
            exit(1);
        }

        adhoc_logger_log(i, "[NODE:%u] INIT: role=%s gateway_no=%u",
                         node->node_id,
                         node->role == ADHOC_ROLE_GATEWAY ? "GATEWAY" : "BEACON",
                         node->gateway_no);
    }
}

/** @brief 为所有节点创建 Win32 线程 */
static void sim_start_threads(void)
{
    uint8_t i;

    for (i = 0u; i < g_sim.node_count; ++i)
    {
        sim_node_t *node = &g_sim.nodes[i];
        InterlockedExchange(&node->running, 1L);
        InterlockedExchange(&node->stop, 0L);
        node->thread = CreateThread(NULL, 0, node_thread, node, 0, NULL);
        if (node->thread == NULL)
        {
            printf("[FATAL] failed to create thread for node %u\n", node->node_id);
            exit(1);
        }
    }
}

static void sim_stop_threads(void)
{
    uint8_t i;

    for (i = 0u; i < g_sim.node_count; ++i)
    {
        InterlockedExchange(&g_sim.nodes[i].stop, 1L);
    }

    for (i = 0u; i < g_sim.node_count; ++i)
    {
        if (g_sim.nodes[i].thread != NULL)
        {
            WaitForSingleObject(g_sim.nodes[i].thread, 5000);
            CloseHandle(g_sim.nodes[i].thread);
            g_sim.nodes[i].thread = NULL;
        }
    }
}

/** @brief 打印全网状态(含状态分布汇总) */
static void sim_print_status(void)
{
    uint8_t i;
    adhoc_node_runtime_status_t status;
    uint8_t c1_count = 0u, un_count = 0u, cn_count = 0u, st1_count = 0u, u1_count = 0u;

    printf("\n===== Simulation Status =====\n");
    for (i = 0u; i < g_sim.node_count; ++i)
    {
        sim_node_t *node = &g_sim.nodes[i];
        memset(&status, 0, sizeof(status));
        adhoc_node_get_runtime_status(node->node_mem, &status);

        switch (status.state)
        {
        case ADHOC_SM_STATE_ST1: st1_count++; break;
        case ADHOC_SM_STATE_U1:  u1_count++;  break;
        case ADHOC_SM_STATE_C1:  c1_count++;  break;
        case ADHOC_SM_STATE_UN:  un_count++;  break;
        case ADHOC_SM_STATE_CN:  cn_count++;  break;
        default: break;
        }

        printf(" [%2u] id=%-8u %s state=%-3s lev=%-2u up=%-8u up_no=%u retry=%-3u",
               i, node->node_id,
               node->role == ADHOC_ROLE_GATEWAY ? "GW " : "BCN",
               state_name(status.state),
               status.joined_level,
               status.upstream_id,
               status.upstream_no,
               status.retry_count);

        if (node->role == ADHOC_ROLE_GATEWAY)
        {
            printf(" locked=%u", status.gateway_network_locked);
        }
        printf("\n");
    }
    printf("--- Summary: ST1=%u U1=%u C1=%u UN=%u CN=%u ---\n",
           st1_count, u1_count, c1_count, un_count, cn_count);
    printf("=============================\n\n");
}

/**
 * @brief 仿真入口: 初始化→启动节点线程→主循环(定时打印状态)→停止→输出统计
 */
int main(void)
{
    uint32_t start_us;
    uint32_t elapsed_s;
    uint32_t last_report_s;

    printf("=== Ad-Hoc-lib Multi-Node Simulation ===\n");
    printf("Gateways: %u  Beacons: %u  Total: %u  Duration: %us  RSSI: %d\n",
           SCENARIO_GW_COUNT, SCENARIO_BCN_COUNT, SCENARIO_NODES,
           SIM_RUN_DURATION_S, SCENARIO_RSSI);
    printf("\n");

    sim_init_scenario();
    sim_start_threads();

    start_us = adhoc_virtual_time_now_us(&g_sim.vt);
    last_report_s = 0u;

    while (1u)
    {
        Sleep(1000);
        elapsed_s = (uint32_t)((adhoc_virtual_time_now_us(&g_sim.vt) - start_us) / 1000000u);

        if (elapsed_s - last_report_s >= 5u || elapsed_s >= SIM_RUN_DURATION_S)
        {
            last_report_s = elapsed_s;
            printf("[T+%2us] ", elapsed_s);
            sim_print_status();
        }

        if (elapsed_s >= SIM_RUN_DURATION_S)
        {
            break;
        }
    }

    sim_stop_threads();

    printf("\n=== Simulation Complete ===\n");
    sim_print_status();

    {
        uint8_t i;
        uint32_t total_acked = 0u, total_retry = 0u;
        for (i = 0u; i < g_sim.node_count; ++i)
        {
            char path[512];
            snprintf(path, sizeof(path), "logs\\node_%u_%u.log", i, g_sim.nodes[i].node_id);
            FILE *f = fopen(path, "r");
            if (f)
            {
                char line[256];
                while (fgets(line, sizeof(line), f))
                {
                    if (strstr(line, "DATA ACKED")) total_acked++;
                    if (strstr(line, "DATA RETRY_EXHAUSTED")) total_retry++;
                }
                fclose(f);
            }
        }
        printf("\n=== Data Summary: ACKED=%u  RETRY_EXHAUSTED=%u ===\n", total_acked, total_retry);

        if (SCENARIO_MODE == 7u)
        {
            printf("\n=== Forward Latency by Node Level ===\n");
            for (i = 0u; i < g_sim.node_count; ++i)
            {
                char path[512];
                snprintf(path, sizeof(path), "logs\\node_%u_%u.log", i, g_sim.nodes[i].node_id);
                FILE *f = fopen(path, "r");
                if (!f) continue;
                uint32_t sum_lat = 0u, count = 0u, max_lat = 0u, min_lat = 999999u;
                char line[256];
                while (fgets(line, sizeof(line), f))
                {
                    const char *p = strstr(line, "latency=");
                    if (p) {
                        uint32_t lat = (uint32_t)atoi(p + 8);
                        sum_lat += lat;
                        if (lat > max_lat) max_lat = lat;
                        if (lat < min_lat) min_lat = lat;
                        count++;
                    }
                }
                fclose(f);
                if (count > 0u)
                {
                    uint32_t node_level = g_sim.nodes[i].role == ADHOC_ROLE_GATEWAY ? 0u : (uint32_t)(i);
                    printf("  Node[%u] id=%u level=%u  avg=%ums min=%ums max=%ums (n=%u)\n",
                           i, g_sim.nodes[i].node_id, node_level,
                           sum_lat / count, min_lat, max_lat, count);
                }
            }
        }
    }

    adhoc_channel_deinit(&g_sim.channel);
    adhoc_logger_shutdown();

    printf("\nLogs written to test/logs/\n");
    return 0;
}
