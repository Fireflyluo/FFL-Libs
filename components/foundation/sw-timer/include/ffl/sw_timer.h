#ifndef FFL_SW_TIMER_H
#define FFL_SW_TIMER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file ffl/sw_timer.h
 * @brief 基于时间轮的软件定时器 C/C++ 兼容公开接口。
 *
 * `ffl_sw_timer_tick_isr()` 应由系统 tick 中断调用，`ffl_sw_timer_process()` 应在主循环或
 * 任务上下文调用。组件不绑定 MCU、RTOS 或具体临界区实现，平台保护通过钩子注入。
 */

/**
 * @brief 软件定时器到期回调函数类型定义
 *
 * @param arg 用户传递给定时器的参数指针
 */
typedef void (*ffl_sw_timer_expired_fn)(void *arg);
typedef void (*ffl_sw_timer_lock_fn)(void);
typedef void (*ffl_sw_timer_unlock_fn)(void);

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
typedef struct ffl_sw_timer
{
    uint8_t active;           /**< 定时器激活状态标志 */
    uint8_t periodic;         /**< 周期性定时器标志 */
    uint16_t rounds;          /**< 剩余轮数 */
    uint16_t slot;            /**< 时间轮槽位索引 */
    uint32_t period_ticks;    /**< 周期时间(单位: ticks) */
    ffl_sw_timer_expired_fn cb; /**< 回调函数指针 */
    void *arg;                /**< 回调函数参数 */
    struct ffl_sw_timer *next; /**< 链表下一个节点 */
} ffl_sw_timer_t;

/**
 * @brief 初始化时间轮
 *
 * @param tick_ms 系统tick的时间间隔(毫秒)，最小值为1ms
 *
 * @note 必须在使用其他定时器函数前调用此函数进行初始化
 */
void ffl_sw_timer_wheel_init(uint32_t tick_ms);

/**
 * @brief 设置临界区进入/退出钩子函数
 *
 * @param lock_fn 进入临界区函数，可为NULL
 * @param unlock_fn 退出临界区函数，可为NULL
 *
 * @note 若未设置，软件定时器在单线程场景下可正常工作；若在中断/多线程环境下使用，
 *       建议提供平台相关的临界区保护函数（如 __disable_irq/__enable_irq）。
 */
void ffl_sw_timer_set_lock_hooks(ffl_sw_timer_lock_fn lock_fn,
                                 ffl_sw_timer_unlock_fn unlock_fn);

/**
 * @brief 启动软件定时器
 *
 * @param timer 定时器结构体指针
 * @param delay_ms 延迟时间(毫秒)，0表示立即触发
 * @param period_ms 周期时间(毫秒)，0表示单次定时器
 * @param expired_fn 定时器到期回调函数
 * @param arg 回调函数参数
 *
 * @return int 返回值:
 *         - 0: 成功
 *         - -1: 参数错误(timer或expired_fn为NULL)
 *         - -2: 未初始化(g_tick_ms为0)
 *
 * @note 如果定时器已经在运行，会先停止再重新启动
 */
int ffl_sw_timer_start(ffl_sw_timer_t *timer,
                       uint32_t delay_ms,
                       uint32_t period_ms,
                       ffl_sw_timer_expired_fn expired_fn,
                       void *arg);

/**
 * @brief 停止软件定时器
 *
 * @param timer 要停止的定时器结构体指针
 *
 * @note 如果定时器未激活或为NULL，函数直接返回
 */
void ffl_sw_timer_stop(ffl_sw_timer_t *timer);

/**
 * @brief 定时器tick中断服务函数
 *
 * @note 此函数应在系统定时器中断中调用，通常每tick_ms毫秒调用一次，
 *       负责推进时间轮指针并检查到期的定时器
 */
void ffl_sw_timer_tick_isr(void);

/**
 * @brief 处理到期的定时器
 *
 * @note 此函数应在主循环或任务中调用，负责执行到期定时器的回调函数。
 *       对于周期性定时器，会自动重新插入时间轮。
 */
void ffl_sw_timer_process(void);

#ifdef __cplusplus
}
#endif

#endif /* FFL_SW_TIMER_H */
