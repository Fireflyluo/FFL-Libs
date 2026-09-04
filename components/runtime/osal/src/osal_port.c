#include "timer.h"

static osal_critical_hook_t g_enter_critical_hook;
static osal_critical_hook_t g_exit_critical_hook;
static osal_tick_hook_t g_tick_init_hook;
static osal_tick_hook_t g_tick_start_hook;
static osal_tick_hook_t g_tick_stop_hook;

void osal_port_set_critical_hooks(osal_critical_hook_t enter_hook, osal_critical_hook_t exit_hook)
{
    g_enter_critical_hook = enter_hook;
    g_exit_critical_hook = exit_hook;
}

void osal_port_enter_critical(void)
{
    if (g_enter_critical_hook != NULL) {
        g_enter_critical_hook();
    }
}

void osal_port_exit_critical(void)
{
    if (g_exit_critical_hook != NULL) {
        g_exit_critical_hook();
    }
}

void osal_port_set_tick_hooks(osal_tick_hook_t init_hook, osal_tick_hook_t start_hook, osal_tick_hook_t stop_hook)
{
    g_tick_init_hook = init_hook;
    g_tick_start_hook = start_hook;
    g_tick_stop_hook = stop_hook;
}

void osal_port_tick_init(void)
{
    if (g_tick_init_hook != NULL) {
        g_tick_init_hook();
    }
}

void osal_port_tick_start(void)
{
    if (g_tick_start_hook != NULL) {
        g_tick_start_hook();
    }
}

void osal_port_tick_stop(void)
{
    if (g_tick_stop_hook != NULL) {
        g_tick_stop_hook();
    }
}
