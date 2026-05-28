/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_link.h
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            链路层抽象接口定义
**                          定义协议层与链路层之间的操作契约, 协议层仅依赖"帧收发+时间+随机"抽象
**--------------------------------------------------------------------------------------------------------
*/

#ifndef ADHOC_LINK_H
#define ADHOC_LINK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 链路操作返回码: 成功 */
#define ADHOC_LINK_OK        0
/** @brief 链路操作返回码: 无帧可收 */
#define ADHOC_LINK_RX_EMPTY  1
/** @brief 链路操作返回码: 参数非法 */
#define ADHOC_LINK_EINVAL   -1
/** @brief 链路操作返回码: 链路忙(发送冲突) */
#define ADHOC_LINK_EBUSY    -2
/** @brief 链路操作返回码: IO 错误 */
#define ADHOC_LINK_EIO      -3

/**
 * @brief 链路层操作接口(虚函数表)
 *
 * 协议层仅通过此结构体与链路层交互, 不直接访问 RF 硬件寄存器。
 * 频点、信道切换、跳频等链路策略必须封装在具体适配层与 ctx 内。
 */
typedef struct
{
    /**
     * @brief 初始化链路资源(可重复调用)
     * @param ctx 链路上下文
     * @return ADHOC_LINK_OK 成功
     */
    int (*init)(void *ctx);

    /**
     * @brief 启动接收(可能在每个协议轮询周期调用)
     * @param ctx 链路上下文
     * @return ADHOC_LINK_OK 成功
     */
    int (*start_rx)(void *ctx);

    /**
     * @brief 发送单帧
     * @param ctx 链路上下文
     * @param buf 帧数据(32字节)
     * @param len 帧长度
     * @return ADHOC_LINK_OK 成功, ADHOC_LINK_EBUSY 忙, ADHOC_LINK_EIO 失败
     */
    int (*tx)(void *ctx, const uint8_t *buf, uint16_t len);

    /**
     * @brief 轮询接收
     * @param ctx  链路上下文
     * @param buf  输出帧缓冲区
     * @param len  输入/输出帧长度
     * @param rssi 输出 RSSI(dBm)
     * @return ADHOC_LINK_OK 有帧, ADHOC_LINK_RX_EMPTY 无帧
     */
    int (*poll_rx)(void *ctx, uint8_t *buf, uint16_t *len, int8_t *rssi);

    /**
     * @brief 获取单调递增微秒时间
     * @param ctx 链路上下文
     * @return 当前微秒时间戳(uint32_t 回绕安全)
     */
    uint32_t (*now_us)(void *ctx);

    /**
     * @brief 获取16位随机数
     * @param ctx 链路上下文
     * @return 随机数(用于时隙扰动等, 不要求密码学质量)
     */
    uint16_t (*rand_u16)(void *ctx);
} adhoc_link_ops_t;

#ifdef __cplusplus
}
#endif

#endif /* ADHOC_LINK_H */
