// osal_pt.c
#include "osal_pt.h"
#include <string.h>

// 全局调度器实例
static osal_pt_scheduler_t g_pt_scheduler;

/**
 * @brief 初始化协程调度器
 */
void osal_pt_scheduler_init(osal_pt_scheduler_t *sched)
{
    if (sched == NULL)
        return;

    sched->pt_list = NULL;
    sched->current_pt = NULL;
    sched->next_pt_id = 1;
    sched->system_time = 0;

    // 保存全局引用
    if (sched != &g_pt_scheduler)
    {
        memcpy(&g_pt_scheduler, sched, sizeof(osal_pt_scheduler_t));
    }
}

/**
 * @brief 创建协程
 */
uint8_t osal_pt_create(osal_pt_scheduler_t *sched,
                       char (*entry)(osal_pt_t *, void *),
                       void *arg,
                       const char *name)
{
    if (sched == NULL || entry == NULL)
    {
        return 0;
    }

    // 分配协程控制块（在实际项目中可能需要内存池管理）
    osal_pt_t *new_pt = (osal_pt_t *)osal_mem_alloc(sizeof(osal_pt_t));
    if (new_pt == NULL)
    {
        return 0;
    }

    // 初始化协程
    PT_INIT(&new_pt->pt);
    new_pt->id = sched->next_pt_id++;
    new_pt->state = PT_STATE_READY;
    new_pt->wakeup_time = 0;
    new_pt->wait_events = 0;
    new_pt->triggered_events = 0;
    new_pt->entry = entry;
    new_pt->arg = arg;
    new_pt->next = NULL;

    // 复制名称
    if (name != NULL)
    {
        strncpy(new_pt->name, name, sizeof(new_pt->name) - 1);
        new_pt->name[sizeof(new_pt->name) - 1] = '\0';
    }
    else
    {
        snprintf(new_pt->name, sizeof(new_pt->name), "pt%d", new_pt->id);
    }

    // 添加到链表
    new_pt->next = sched->pt_list;
    sched->pt_list = new_pt;

    // printf("Protothread created: %s (ID: %d)\n", new_pt->name, new_pt->id);
    return new_pt->id;
}

/**
 * @brief 调度所有协程
 */
void osal_pt_schedule(osal_pt_scheduler_t *sched)
{
    if (sched == NULL)
        return;

    // 更新系统时间
    sched->system_time = osal_GetSystemClock();

    osal_pt_t *pt = sched->pt_list;
    osal_pt_t *prev = NULL;

    while (pt != NULL)
    {
        // 检查协程是否可运行
        uint8_t can_run = 0;

        switch (pt->state)
        {
        case PT_STATE_READY:
            can_run = 1;
            break;

        case PT_STATE_WAITING:
            // 检查事件是否触发或延时是否到期
            if ((pt->wait_events && (pt->triggered_events & pt->wait_events)) ||
                (pt->wakeup_time && sched->system_time >= pt->wakeup_time))
            {
                can_run = 1;
            }
            break;

        case PT_STATE_EXITED:
            // 移除已退出的协程
            osal_pt_t *to_remove = pt;
            if (prev)
            {
                prev->next = pt->next;
            }
            else
            {
                sched->pt_list = pt->next;
            }
            pt = pt->next;

            // printf("Protothread removed: %s\n", to_remove->name);
            osal_mem_free(to_remove);
            continue;

        default:
            break;
        }

        if (can_run)
        {
            sched->current_pt = pt;
            pt->state = PT_STATE_RUNNING;

            // 执行协程
            char result = pt->entry(pt, pt->arg);

            if (result == 0)
            {
                // 协程主动让出
                pt->state = PT_STATE_WAITING;
            }
            else if (result == 2)
            {
                // 协程退出
                pt->state = PT_STATE_EXITED;
            }
            else
            {
                // 协程执行完成本次时间片
                pt->state = PT_STATE_READY;
            }

            sched->current_pt = NULL;
        }

        prev = pt;
        pt = pt->next;
    }
}

/**
 * @brief 设置协程事件
 */
void osal_pt_set_event(osal_pt_t *pt, uint16_t events)
{
    if (pt == NULL)
        return;

    pt->triggered_events |= events;
}

/**
 * @brief 协程延时
 */
void osal_pt_delay(osal_pt_t *pt, uint32_t ms)
{
    if (pt == NULL || pt->sched == NULL)
        return;

    pt->wakeup_time = pt->sched->system_time + ms;
}

/**
 * @brief 等待事件（基础版本）
 */
uint8_t osal_pt_wait_event(osal_pt_t *pt, uint16_t event_mask)
{
    if (pt == NULL)
        return 0;

    pt->wait_events = event_mask;
    if (pt->triggered_events & event_mask)
    {
        pt->triggered_events &= ~event_mask;
        pt->wait_events = 0;
        return 1; // 事件已就绪
    }

    return 0; // 需要等待
}