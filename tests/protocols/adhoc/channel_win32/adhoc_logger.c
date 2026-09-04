/*-----------------------------------------------File Info------------------------------------------------
** File Name:               adhoc_logger.c
** Created date:            2026.5.14
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            仿真日志系统实现
**                          全局单例, slot 索引管理多个日志文件,
**                          每行自动附加系统时间戳, CriticalSection 线程安全
**--------------------------------------------------------------------------------------------------------
*/

#include "adhoc_logger.h"

#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <windows.h>

/** @brief 全局日志临界区 */
static CRITICAL_SECTION g_log_lock;
/** @brief 临界区是否已初始化 */
static uint8_t         g_log_lock_inited = 0u;
/** @brief 日志目录路径副本 */
static char            g_logs_dir_buf[512];
/** @brief 日志目录路径指针(指向 g_logs_dir_buf) */
static const char     *g_logs_dir = NULL;
/** @brief slot→文件映射表 */
static FILE           *g_log_files[ADHOC_LOGGER_MAX_SLOTS];

/**
 * @brief 初始化日志系统(全局单例)
 */
void adhoc_logger_init(const char *logs_dir)
{
    uint8_t i;

    if (g_log_lock_inited == 0u)
    {
        InitializeCriticalSection(&g_log_lock);
        g_log_lock_inited = 1u;
    }

    for (i = 0u; i < ADHOC_LOGGER_MAX_SLOTS; ++i)
    {
        g_log_files[i] = NULL;
    }

    strncpy(g_logs_dir_buf, logs_dir, sizeof(g_logs_dir_buf) - 1u);
    g_logs_dir_buf[sizeof(g_logs_dir_buf) - 1u] = '\0';
    g_logs_dir = g_logs_dir_buf;
}

/**
 * @brief 为指定槽位打开日志文件(w 模式, 自动创建目录)
 */
void adhoc_logger_open_slot(uint8_t slot_idx, const char *file_name)
{
    char path[512];

    if (g_logs_dir == NULL || file_name == NULL || slot_idx >= ADHOC_LOGGER_MAX_SLOTS)
    {
        return;
    }

    CreateDirectoryA(g_logs_dir, NULL);
    snprintf(path, sizeof(path), "%s\\%s.log", g_logs_dir, file_name);

    EnterCriticalSection(&g_log_lock);
    if (g_log_files[slot_idx] != NULL)
    {
        fclose(g_log_files[slot_idx]);
    }
    g_log_files[slot_idx] = fopen(path, "w");
    LeaveCriticalSection(&g_log_lock);
}

/**
 * @brief 关闭指定槽位的日志文件
 */
void adhoc_logger_close_slot(uint8_t slot_idx)
{
    if (slot_idx >= ADHOC_LOGGER_MAX_SLOTS)
    {
        return;
    }

    EnterCriticalSection(&g_log_lock);
    if (g_log_files[slot_idx] != NULL)
    {
        fclose(g_log_files[slot_idx]);
        g_log_files[slot_idx] = NULL;
    }
    LeaveCriticalSection(&g_log_lock);
}

/**
 * @brief 向指定槽位写入格式化日志(自动加 [HH:MM:SS.ms] 时间戳和换行)
 */
void adhoc_logger_log(uint8_t slot_idx, const char *format, ...)
{
    FILE *f;
    SYSTEMTIME st;
    va_list args;

    if (slot_idx >= ADHOC_LOGGER_MAX_SLOTS || format == NULL)
    {
        return;
    }

    EnterCriticalSection(&g_log_lock);
    f = g_log_files[slot_idx];
    if (f != NULL)
    {
        GetLocalTime(&st);
        fprintf(f, "[%02u:%02u:%02u.%03u] ",
                st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
        va_start(args, format);
        vfprintf(f, format, args);
        va_end(args);
        fprintf(f, "\n");
        fflush(f);
    }
    LeaveCriticalSection(&g_log_lock);
}

/**
 * @brief 关闭所有日志文件并释放临界区
 */
void adhoc_logger_shutdown(void)
{
    uint8_t i;

    EnterCriticalSection(&g_log_lock);
    for (i = 0u; i < ADHOC_LOGGER_MAX_SLOTS; ++i)
    {
        if (g_log_files[i] != NULL)
        {
            fclose(g_log_files[i]);
            g_log_files[i] = NULL;
        }
    }
    LeaveCriticalSection(&g_log_lock);

    if (g_log_lock_inited != 0u)
    {
        DeleteCriticalSection(&g_log_lock);
        g_log_lock_inited = 0u;
    }
}
