#ifndef ADHOC_LINK_H
#define ADHOC_LINK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADHOC_LINK_OK        0
#define ADHOC_LINK_RX_EMPTY  1
#define ADHOC_LINK_EINVAL   -1
#define ADHOC_LINK_EBUSY    -2
#define ADHOC_LINK_EIO      -3

typedef struct
{
    /* 协议层仅依赖“帧收发+时间+随机”抽象。
       频点、信道切换、跳频等链路策略必须封装在具体适配层与 ctx 内。 */
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
