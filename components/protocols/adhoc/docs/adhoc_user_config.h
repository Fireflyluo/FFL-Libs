#ifndef ADHOC_USER_CONFIG_H
#define ADHOC_USER_CONFIG_H

/*
 * Ad-Hoc-lib user config template
 *
 * 用法：
 * 1) 复制本文件到你的工程目录（建议命名：adhoc_user_config.h）
 * 2) 按需取消注释并修改下面的宏
 * 3) 在编译选项里增加：
 *      -DADHOC_CONFIG_USER_HEADER=\"adhoc_user_config.h\"
 *
 * 说明：
 * - 本文件中的宏会覆盖 Ad-Hoc-lib/include/adhoc_config.h 的默认值
 * - 仅修改你需要的项；未定义项会继续使用默认值
 */

/* =========================
 * 1) Timing
 * ========================= */
/* #define ADHOC_CONFIG_TMOS_TICK_US 625u */
/* #define ADHOC_CONFIG_TIMING_M_RECOMMENDED_MIN 10u */
/* #define ADHOC_CONFIG_TIMING_M_RECOMMENDED_MAX 1000u */

/* =========================
 * 2) State Machine (SM)
 * ========================= */
/* #define ADHOC_CONFIG_SM_GATEWAY_ID_MAX 1000000u */
/* #define ADHOC_CONFIG_SM_BEACON_ID_MIN 1000001u */
/* #define ADHOC_CONFIG_SM_DEFAULT_RETRY_MAX 30u */
/* #define ADHOC_CONFIG_SM_DEFAULT_NETWORK_WINDOW_US 30000000u */
/* #define ADHOC_CONFIG_SM_RSSI_STRONG_MIN_DBM (-65) */
/* #define ADHOC_CONFIG_SM_RSSI_MEDIUM_MIN_DBM (-80) */
/* #define ADHOC_CONFIG_SM_JOIN_LEVEL_MAX 254u */
/* #define ADHOC_CONFIG_SM_UPSTREAM_BINDING_CAPACITY 6u */  /* 1~8 */
/* #define ADHOC_CONFIG_SM_NEIGHBOR_CACHE_CAPACITY 6u */    /* >0 */
/* #define ADHOC_CONFIG_SM_UPSTREAM_LOSS_CYCLES 3u */
/* #define ADHOC_CONFIG_SM_DOWNSTREAM_BINDING_CAPACITY 24u */ /* >0，直接下级绑定记录容量 */

/* 测试开关：1=丢弃 join confirm（仅用于测试） */
/* #define ADHOC_CONFIG_TEST_DROP_JOIN_CONFIRM 0u */

/* =========================
 * 3) Reply/Ack list
 * ========================= */
/* #define ADHOC_CONFIG_REPLY_LIST_CAPACITY 24u */
/* #define ADHOC_CONFIG_REPLY_MAX_PER_FRAME 5u */           /* >0 */

/* =========================
 * 4) Data plane
 * ========================= */
/* #define ADHOC_CONFIG_DATA_DEDUP_CAPACITY 64u */          /* >0 */
/* #define ADHOC_CONFIG_DATA_ACK_QUEUE_CAPACITY 32u */      /* >0 */
/* #define ADHOC_CONFIG_DATA_TX_QUEUE_CAPACITY 16u */       /* >0 */
/* #define ADHOC_CONFIG_DATA_TX_REPORT_QUEUE_CAPACITY 16u */ /* >0 */
/* #define ADHOC_CONFIG_DATA_TX_RETRY_MAX 30u */
/* #define ADHOC_CONFIG_DATA_DEFAULT_DEDUP_WINDOW_MS (6u * 60u * 60u * 1000u) */
/* #define ADHOC_CONFIG_DATA_GATEWAY_ID_MAX 1000000u */

/* =========================
 * 5) CRC8 LUT storage
 * =========================
 * 如需把 CRC8 查表放入指定段（例如 CH32 的非零等待区）可启用：
 */
/* #define ADHOC_CONFIG_CRC8_TABLE_STORAGE __attribute__((section(".flash1_rodata"))) */

#endif /* ADHOC_USER_CONFIG_H */
