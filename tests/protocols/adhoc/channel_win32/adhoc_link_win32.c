/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_link_win32.c
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            Windows 链路适配实现
**                          实现 adhoc_link_ops_t 的全部六个接口,
**                          将协议层的帧收发请求转换为信道操作, 并记录 TX/RX 日志
**--------------------------------------------------------------------------------------------------------
*/

#include "adhoc_link_win32.h"
#include "adhoc_logger.h"

#include <stdlib.h>
#include <string.h>

/** @brief 类型安全上下文转换 */
static adhoc_link_win32_ctx_t *cast_ctx(void *ctx)
{
    return (adhoc_link_win32_ctx_t *)ctx;
}

/**
 * @brief 链路初始化: 仅记录日志
 */
static int link_init(void *ctx)
{
    adhoc_link_win32_ctx_t *lctx = cast_ctx(ctx);
    if (lctx == NULL)
    {
        return ADHOC_LINK_EINVAL;
    }
    adhoc_logger_log(lctx->log_slot, "[NODE:%u] link_init", lctx->node_id);
    return ADHOC_LINK_OK;
}

/**
 * @brief 启动接收: 信道始终就绪, 无操作
 */
static int link_start_rx(void *ctx)
{
    (void)ctx;
    return ADHOC_LINK_OK;
}

/**
 * @brief 发送帧: 代理到 adhoc_channel_tx, 记录 TX 日志
 */
static int link_tx(void *ctx, const uint8_t *buf, uint16_t len)
{
    adhoc_link_win32_ctx_t *lctx = cast_ctx(ctx);

    if (lctx == NULL || buf == NULL)
    {
        return ADHOC_LINK_EINVAL;
    }
    if (len != ADHOC_FRAME_LEN)
    {
        return ADHOC_LINK_EINVAL;
    }

    if (!adhoc_channel_tx(lctx->channel, lctx->node_idx, buf, (uint8_t)len))
    {
        return ADHOC_LINK_EBUSY;
    }

    adhoc_logger_log(lctx->log_slot, "[NODE:%u] TX class=%c level=%u",
                     lctx->node_id, (buf[0] & 0x80u) ? 'D' : 'A', buf[1]);

    return ADHOC_LINK_OK;
}

/**
 * @brief 轮询接收: 代理到 adhoc_channel_poll_rx, 记录 RX 日志
 */
static int link_poll_rx(void *ctx, uint8_t *buf, uint16_t *len, int8_t *rssi)
{
    adhoc_link_win32_ctx_t *lctx = cast_ctx(ctx);
    uint8_t rx_len = 0u;

    if (lctx == NULL || buf == NULL || rssi == NULL)
    {
        return ADHOC_LINK_EINVAL;
    }

    if (!adhoc_channel_poll_rx(lctx->channel, lctx->node_idx, buf, &rx_len, rssi))
    {
        if (len != NULL)
        {
            *len = 0u;
        }
        return ADHOC_LINK_RX_EMPTY;
    }

    if (len != NULL)
    {
        *len = (uint16_t)rx_len;
    }

    adhoc_logger_log(lctx->log_slot, "[NODE:%u] RX class=%c level=%u rssi=%d",
                     lctx->node_id, (buf[0] & 0x80u) ? 'D' : 'A', buf[1], *rssi);

    return ADHOC_LINK_OK;
}

/**
 * @brief 获取微秒时间: 代理到虚拟时间
 */
static uint32_t link_now_us(void *ctx)
{
    adhoc_link_win32_ctx_t *lctx = cast_ctx(ctx);
    if (lctx == NULL || lctx->vt == NULL)
    {
        return 0u;
    }
    return adhoc_virtual_time_now_us(lctx->vt);
}

/**
 * @brief 生成随机数: rand() + 线性同余混合
 */
static uint16_t link_rand_u16(void *ctx)
{
    adhoc_link_win32_ctx_t *lctx = cast_ctx(ctx);
    uint16_t r;

    if (lctx == NULL)
    {
        return (uint16_t)(rand() & 0xFFFFu);
    }

    r = (uint16_t)(rand() & 0xFFFFu);
    lctx->rand_state = (lctx->rand_state * 1103515245u + 12345u) ^ (uint32_t)r;
    return (uint16_t)(lctx->rand_state & 0xFFFFu);
}

/** @brief 全局 Windows 链路操作接口实例 */
const adhoc_link_ops_t g_adhoc_link_win32_ops = {
    link_init,
    link_start_rx,
    link_tx,
    link_poll_rx,
    link_now_us,
    link_rand_u16
};

/**
 * @brief 初始化链路适配上下文
 */
void adhoc_link_win32_ctx_init(adhoc_link_win32_ctx_t *ctx,
                               adhoc_channel_t *channel, uint8_t node_idx,
                               uint8_t log_slot,
                               adhoc_virtual_time_t *vt,
                               uint32_t node_id)
{
    if (ctx == NULL)
    {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    ctx->channel    = channel;
    ctx->node_idx   = node_idx;
    ctx->log_slot   = log_slot;
    ctx->vt         = vt;
    ctx->node_id    = node_id;
    ctx->rand_state = (uint32_t)(node_id ^ 0xA5A55A5Au);
}
