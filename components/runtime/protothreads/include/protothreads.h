// protothreads.h
#ifndef PROTOTHREADS_H
#define PROTOTHREADS_H

#include <stdint.h>

/**
 * @brief Protothread 状态结构
 * 仅保存行号，极其轻量
 */
typedef struct pt
{
    uint16_t lc; // 行号计数器（保存switch case的位置）
} pt_t;

/**
 * @brief 初始化protothread
 */
#define PT_INIT(pt) ((pt)->lc = 0)

/**
 * @brief 开始protothread定义
 */
#define PT_BEGIN(pt)      \
    {                     \
        switch ((pt)->lc) \
        {                 \
        case 0:

/**
 * @brief 结束protothread定义
 */
#define PT_END(pt) \
    }              \
    (pt)->lc = 0;  \
    return 0;      \
    }

/**
 * @brief 等待条件满足
 */
#define PT_WAIT_UNTIL(pt, cond) \
    do                          \
    {                           \
        (pt)->lc = __LINE__;    \
    case __LINE__:              \
        if (!(cond))            \
            return 0;           \
    } while (0)

/**
 * @brief 等待条件不满足
 */
#define PT_WAIT_WHILE(pt, cond) PT_WAIT_UNTIL((pt), !(cond))

/**
 * @brief 让出执行权
 */
#define PT_YIELD(pt)         \
    do                       \
    {                        \
        (pt)->lc = __LINE__; \
        return 0;            \
    case __LINE__:;          \
    } while (0)

/**
 * @brief 等待子protothread完成
 */
#define PT_SPAWN(pt, child, thread) \
    do                              \
    {                               \
        (pt)->lc = __LINE__;        \
    case __LINE__:                  \
        if (!(thread))              \
            return 0;               \
    } while (0)

/**
 * @brief 退出protothread
 */
#define PT_EXIT(pt)   \
    do                \
    {                 \
        (pt)->lc = 0; \
        return 2;     \
    } while (0)

/**
 * @brief 重启protothread
 */
#define PT_RESTART(pt) \
    do                 \
    {                  \
        (pt)->lc = 0;  \
        PT_INIT(pt);   \
    } while (0)

/**
 * @brief 检查protothread是否结束
 */
#define PT_SCHEDULE(f) ((f) < 2)

/**
 * @brief 定义protothread函数类型
 */
#define PT_THREAD(name_args) char name_args

#endif /* PROTOTHREADS_H */