#include "ffl/sw_timer.h"

#include <stddef.h>
#include <string.h>

/**
 * @file ffl_sw_timer.c
 * @brief 基于固定 256 槽时间轮的软件定时器实现。
 */

#define FFL_SW_TIMER_WHEEL_SIZE 256U

static ffl_sw_timer_t *g_wheel[FFL_SW_TIMER_WHEEL_SIZE];
static ffl_sw_timer_t *g_expired_list;
static uint16_t g_current_slot;
static uint32_t g_tick_ms;

static ffl_sw_timer_lock_fn g_lock_fn;
static ffl_sw_timer_unlock_fn g_unlock_fn;

static void ffl_sw_timer_enter_critical(void)
{
    if (g_lock_fn != NULL)
    {
        g_lock_fn();
    }
}

static void ffl_sw_timer_exit_critical(void)
{
    if (g_unlock_fn != NULL)
    {
        g_unlock_fn();
    }
}

static uint32_t ffl_sw_timer_ms_to_ticks(uint32_t ms)
{
    uint32_t ticks;

    if (ms == 0U)
    {
        return 0U;
    }

    ticks = ms / g_tick_ms;
    if ((ms % g_tick_ms) != 0U)
    {
        ticks++;
    }

    return (ticks == 0U) ? 1U : ticks;
}

static void ffl_sw_timer_wheel_insert(ffl_sw_timer_t *timer, uint32_t ticks)
{
    uint32_t slot_offset = ticks % FFL_SW_TIMER_WHEEL_SIZE;

    timer->rounds = (uint16_t)((ticks - 1U) / FFL_SW_TIMER_WHEEL_SIZE);
    timer->slot = (uint16_t)((g_current_slot + slot_offset) % FFL_SW_TIMER_WHEEL_SIZE);
    timer->next = g_wheel[timer->slot];
    g_wheel[timer->slot] = timer;
    timer->active = 1U;
}

void ffl_sw_timer_wheel_init(uint32_t tick_ms)
{
    if (tick_ms == 0U)
    {
        tick_ms = 1U;
    }

    ffl_sw_timer_enter_critical();
    memset(g_wheel, 0, sizeof(g_wheel));
    g_expired_list = NULL;
    g_current_slot = 0U;
    g_tick_ms = tick_ms;
    ffl_sw_timer_exit_critical();
}

void ffl_sw_timer_set_lock_hooks(ffl_sw_timer_lock_fn lock_fn,
                                 ffl_sw_timer_unlock_fn unlock_fn)
{
    g_lock_fn = lock_fn;
    g_unlock_fn = unlock_fn;
}

void ffl_sw_timer_stop(ffl_sw_timer_t *timer)
{
    ffl_sw_timer_t **head;
    ffl_sw_timer_t *prev = NULL;
    ffl_sw_timer_t *node;

    if (timer == NULL || timer->active == 0U)
    {
        return;
    }

    ffl_sw_timer_enter_critical();

    head = &g_wheel[timer->slot];
    node = *head;

    while (node != NULL && node != timer) {
        prev = node;
        node = node->next;
    }

    if (node == timer) {
        if (prev == NULL) {
            *head = node->next;
        }
        else {
            prev->next = node->next;
        }

        timer->next = NULL;
        timer->active = 0U;
        ffl_sw_timer_exit_critical();
        return;
    }

    head = &g_expired_list;
    prev = NULL;
    node = *head;

    while (node != NULL && node != timer) {
        prev = node;
        node = node->next;
    }

    if (node == timer) {
        if (prev == NULL) {
            *head = node->next;
        }
        else {
            prev->next = node->next;
        }

        timer->next = NULL;
        timer->active = 0U;
    }

    ffl_sw_timer_exit_critical();
}

int ffl_sw_timer_start(ffl_sw_timer_t *timer,
                       uint32_t delay_ms,
                       uint32_t period_ms,
                       ffl_sw_timer_expired_fn expired_fn,
                       void *arg)
{
    uint32_t delay_ticks;

    if (timer == NULL || expired_fn == NULL)
    {
        return -1;
    }

    if (g_tick_ms == 0U)
    {
        return -2;
    }

    delay_ticks = ffl_sw_timer_ms_to_ticks(delay_ms);
    timer->period_ticks = (period_ms == 0U) ? 0U : ffl_sw_timer_ms_to_ticks(period_ms);
    timer->periodic = (timer->period_ticks != 0U) ? 1U : 0U;
    timer->cb = expired_fn;
    timer->arg = arg;

    ffl_sw_timer_stop(timer);

    ffl_sw_timer_enter_critical();

    if (delay_ticks == 0U)
    {
        timer->next = g_expired_list;
        g_expired_list = timer;
        timer->active = 1U;
    }
    else
    {
        ffl_sw_timer_wheel_insert(timer, delay_ticks);
    }

    ffl_sw_timer_exit_critical();

    return 0;
}

void ffl_sw_timer_tick_isr(void)
{
    ffl_sw_timer_t *node;
    ffl_sw_timer_t *prev = NULL;
    ffl_sw_timer_t *next;

    g_current_slot = (uint16_t)((g_current_slot + 1U) % FFL_SW_TIMER_WHEEL_SIZE);
    node = g_wheel[g_current_slot];

    while (node != NULL)
    {
        next = node->next;

        if (node->rounds > 0U)
        {
            node->rounds--;
            prev = node;
        }
        else
        {
            if (prev == NULL)
            {
                g_wheel[g_current_slot] = next;
            }
            else
            {
                prev->next = next;
            }

            node->next = g_expired_list;
            g_expired_list = node;
        }

        node = next;
    }
}

void ffl_sw_timer_process(void)
{
    ffl_sw_timer_t *local_list;
    ffl_sw_timer_t *node;

    ffl_sw_timer_enter_critical();
    local_list = g_expired_list;
    g_expired_list = NULL;
    ffl_sw_timer_exit_critical();

    while (local_list != NULL)
    {
        node = local_list;
        local_list = local_list->next;

        node->next = NULL;
        node->active = 0U;

        if (node->cb != NULL)
        {
            node->cb(node->arg);
        }

        if (node->periodic != 0U)
        {
            ffl_sw_timer_enter_critical();
            ffl_sw_timer_wheel_insert(node, node->period_ticks);
            ffl_sw_timer_exit_critical();
        }
    }
}
