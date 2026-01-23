// osal_pt.h
#ifndef OSAL_PT_H
#define OSAL_PT_H

#include "protothreads.h"
#include "type.h"
#include "osal_event.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // 协程状态定义
    typedef enum
    {
        PT_STATE_READY = 0, // 就绪
        PT_STATE_RUNNING,   // 运行中
        PT_STATE_WAITING,   // 等待中
        PT_STATE_EXITED     // 已退出
    } pt_state_t;

    // 协程控制块
    typedef struct osal_pt
    {
        pt_t pt;                   // protothread上下文
        uint8_t id;                // 协程ID
        uint8_t state;             // 状态
        uint32_t wakeup_time;      // 唤醒时间（用于延时）
        uint16_t wait_events;      // 等待的事件掩码
        uint16_t triggered_events; // 已触发的事件
        char name[16];             // 协程名称（调试用）

        // 协程函数指针
        char (*entry)(struct osal_pt *pt, void *arg);
        void *arg; // 参数

        struct osal_pt *next; // 链表指针
    } osal_pt_t;

    // 协程调度器
    typedef struct
    {
        osal_pt_t *pt_list;    // 协程链表
        osal_pt_t *current_pt; // 当前运行的协程
        uint8_t next_pt_id;    // 下一个协程ID
        uint32_t system_time;  // 系统时间
    } osal_pt_scheduler_t;

    // API函数
    void osal_pt_scheduler_init(osal_pt_scheduler_t *sched);
    uint8_t osal_pt_create(osal_pt_scheduler_t *sched,
                           char (*entry)(osal_pt_t *, void *),
                           void *arg,
                           const char *name);
    void osal_pt_schedule(osal_pt_scheduler_t *sched);
    void osal_pt_set_event(osal_pt_t *pt, uint16_t events);
    void osal_pt_delay(osal_pt_t *pt, uint32_t ms);
    uint8_t osal_pt_wait_event(osal_pt_t *pt, uint16_t event_mask);

// 简化宏定义
#define PT_BEGIN_WRAPPER(pt) PT_BEGIN(&(pt)->pt)
#define PT_END_WRAPPER(pt) PT_END(&(pt)->pt)
#define PT_YIELD_WRAPPER(pt) PT_YIELD(&(pt)->pt)
#define PT_WAIT_UNTIL_WRAPPER(pt, cond) PT_WAIT_UNTIL(&(pt)->pt, cond)

// 等待事件宏
#define PT_WAIT_EVENT(pt, event_mask)                                           \
    do                                                                          \
    {                                                                           \
        (pt)->wait_events = (event_mask);                                       \
        PT_WAIT_UNTIL(&(pt)->pt, ((pt)->triggered_events & (pt)->wait_events)); \
        (pt)->triggered_events &= ~(pt)->wait_events;                           \
        (pt)->wait_events = 0;                                                  \
    } while (0)

// 延时宏
#define PT_DELAY(pt, ms)                                                         \
    do                                                                           \
    {                                                                            \
        (pt)->wakeup_time = (pt)->sched->system_time + (ms);                     \
        PT_WAIT_UNTIL(&(pt)->pt, (pt)->sched->system_time >= (pt)->wakeup_time); \
    } while (0)

#ifdef __cplusplus
}
#endif

#endif /* OSAL_PT_H */