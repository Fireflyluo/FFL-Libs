#ifndef __SW_TIMER_H
#define __SW_TIMER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 软件定时器回调函数类型定义
 * 
 * @param arg 用户传递给定时器的参数指针
 */
typedef void (*sw_timer_cb_t)(void *arg);
typedef void (*sw_timer_lock_hook_t)(void);

/**
 * @brief 软件定时器结构体
 * 
 * 时间轮算法的核心数据结构，每个定时器实例包含以下信息：
 * - active: 定时器是否处于激活状态 (1=激活, 0=未激活)
 * - periodic: 是否为周期性定时器 (1=周期性, 0=单次)
 * - rounds: 需要经过的完整轮数
 * - slot: 在时间轮中的槽位索引
 * - period_ticks: 周期性定时器的周期(以tick为单位)
 * - cb: 定时器到期时调用的回调函数
 * - arg: 传递给回调函数的用户参数
 * - next: 链表指针，用于同一槽位内的定时器链表
 */
typedef struct sw_timer
{
    uint8_t active;           /**< 定时器激活状态标志 */
    uint8_t periodic;         /**< 周期性定时器标志 */
    uint16_t rounds;          /**< 剩余轮数 */
    uint16_t slot;            /**< 时间轮槽位索引 */
    uint32_t period_ticks;    /**< 周期时间(单位: ticks) */
    sw_timer_cb_t cb;         /**< 回调函数指针 */
    void *arg;                /**< 回调函数参数 */
    struct sw_timer *next;    /**< 链表下一个节点 */
} sw_timer_t;

/**
 * @brief 初始化时间轮
 * 
 * @param tick_ms 系统tick的时间间隔(毫秒)，最小值为1ms
 * 
 * @note 必须在使用其他定时器函数前调用此函数进行初始化
 */
void sw_timer_wheel_init(uint32_t tick_ms);

/**
 * @brief 设置临界区进入/退出钩子函数
 *
 * @param lock_hook 进入临界区函数，可为NULL
 * @param unlock_hook 退出临界区函数，可为NULL
 *
 * @note 若未设置，sw_timer在单线程场景下可正常工作；若在中断/多线程环境下使用，
 *       建议提供平台相关的临界区保护函数（如 __disable_irq/__enable_irq）。
 */
void sw_timer_set_lock_hooks(sw_timer_lock_hook_t lock_hook, sw_timer_lock_hook_t unlock_hook);

/**
 * @brief 启动软件定时器
 * 
 * @param timer 定时器结构体指针
 * @param delay_ms 延迟时间(毫秒)，0表示立即触发
 * @param period_ms 周期时间(毫秒)，0表示单次定时器
 * @param cb 定时器到期回调函数
 * @param arg 回调函数参数
 * 
 * @return int 返回值:
 *         - 0: 成功
 *         - -1: 参数错误(timer或cb为NULL)
 *         - -2: 未初始化(g_tick_ms为0)
 * 
 * @note 如果定时器已经在运行，会先停止再重新启动
 */
int sw_timer_start(sw_timer_t *timer, uint32_t delay_ms, uint32_t period_ms, sw_timer_cb_t cb, void *arg);

/**
 * @brief 停止软件定时器
 * 
 * @param timer 要停止的定时器结构体指针
 * 
 * @note 如果定时器未激活或为NULL，函数直接返回
 */
void sw_timer_stop(sw_timer_t *timer);

/**
 * @brief 定时器tick中断服务函数
 * 
 * @note 此函数应在系统定时器中断中调用，通常每tick_ms毫秒调用一次
 *       负责推进时间轮指针并检查到期的定时器
 */
void sw_timer_tick_isr(void);

/**
 * @brief 处理到期的定时器
 * 
 * @note 此函数应在主循环或任务中调用，负责执行到期定时器的回调函数
 *       对于周期性定时器，会自动重新插入时间轮
 */
void sw_timer_process(void);

#ifdef __cplusplus
}
#endif

#endif /* __SW_TIMER_H */
