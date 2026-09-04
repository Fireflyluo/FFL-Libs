#include "osal.h"
#include "osal_event.h"
#include "osal_memory.h"

#include <string.h>

osal_msg_q_t osal_qHead;
/*********************************************************************
 * @fn osal_init_system
 *
 * @brief
 *
 *   此函数通过创建任务表中定义的任务来初始化“任务”系统（OSAL_Tasks.h）。
 *
 * @param   void
 *
 * @return  ZSUCCESS
 */
uint8 osal_init_system(void)
{
    // 初始化内存分配系统
    osal_mem_init();

    // 初始化消息队列
    osal_qHead = NULL;

#if defined(OSAL_TOTAL_MEM)
    osal_msg_cnt = 0;
#endif

    osalTimerInit();
    osal_init_TaskHead();

    return (ZSUCCESS);
}

/*********************************************************************
 * @fn osal_start_system
 *
 * @brief
 *
 *   这个函数是任务系统的主循环函数。它会遍历所有任务事件，并调用相应任务的
 *   task_event_processor() 函数处理事件。如果没有任务事件，则该函数会让处理器进入睡眠状态。
 *   本函数不会返回。
 *
 * @param   void
 *
 * @return  none
 */
void osal_start_system(void)
{
    uint16 events;
    uint16 retEvents;

#ifdef OSAL_PT_ENABLE
    osal_pt_scheduler_t sched;
    osal_pt_scheduler_init(&sched); // 初始化协程调度器
#endif

    while (1)
    {
        TaskActive = osalNextActiveTask();
        if (TaskActive)
        {
            HAL_ENTER_CRITICAL_SECTION();
            events = TaskActive->events;
            // 清除此任务的事件标志
            TaskActive->events = 0;
            HAL_EXIT_CRITICAL_SECTION();

            if (events != 0)
            {
                // 调用任务处理事件
                if (TaskActive->pfnEventProcessor)
                {
                    retEvents = (TaskActive->pfnEventProcessor)(TaskActive->taskID, events);

                    // 将未处理完的事件重新添加回当前任务
                    HAL_ENTER_CRITICAL_SECTION();
                    TaskActive->events |= retEvents;
                    HAL_EXIT_CRITICAL_SECTION();
                }
            }
        }
#ifdef OSAL_PT_ENABLE
        // 协程调度
        osal_pt_schedule(&sched);
#endif
    }
}

/*********************************************************************
 * @fn osal_strlen
 *
 * @brief
 *
 *   计算字符串长度。字符串必须以空字符结尾。
 *
 * @param   char *pString - 指向文本字符串的指针
 *
 * @return  int - 字符数量
 */
int osal_strlen(char *pString)
{
    return (int)(strlen(pString));
}

/*********************************************************************
 * @fn osal_memcpy
 *
 * @brief
 *
 *   通用内存拷贝函数。
 *
 *   注意：此函数与标准 memcpy() 不同之处在于，
 *         它返回的是目标缓冲区下一个字节地址。而标准 memcpy() 返回原始的目标地址。
 *
 * @param   dst - 目标地址
 * @param   src - 源地址
 * @param   len - 要复制的字节数
 *
 * @return  指向目标缓冲区末尾的指针
 */
void *osal_memcpy(void *dst, const void *src, unsigned int len)
{
    uint8 *pDst;
    const uint8 *pSrc;

    pSrc = src;
    pDst = dst;

    while (len--)
        *pDst++ = *pSrc++;

    return (pDst);
}

/*********************************************************************
 * @fn osal_revmemcpy
 *
 * @brief   通用反向内存拷贝。从源缓冲区末尾开始拷贝数据，
 *          即先将源地址指针向前移动“len”个字节，然后递减指针进行拷贝。
 *
 *   注意：此函数与标准 memcpy() 不同之处在于，
 *         它返回的是目标缓冲区下一个字节地址。而标准 memcpy() 返回原始的目标地址。
 *
 * @param   dst - 目标地址
 * @param   src - 源地址
 * @param   len - 要复制的字节数
 *
 * @return  指向目标缓冲区末尾的指针
 */
void *osal_revmemcpy(void *dst, const void *src, unsigned int len)
{
    uint8 *pDst;
    const uint8 *pSrc;

    pSrc = src;
    pSrc += (len - 1);
    pDst = dst;

    while (len--)
        *pDst++ = *pSrc--;

    return (pDst);
}

/*********************************************************************
 * @fn osal_memdup
 *
 * @brief   分配一个新缓冲区 [使用 osal_mem_alloc()] 并将源缓冲区内容复制进去。
 *
 * @param   src - 源地址
 * @param   len - 要复制的字节数
 *
 * @return  新分配缓冲区的指针；如果分配失败则返回 NULL。
 */
void *osal_memdup(const void *src, unsigned int len)
{
    uint8 *pDst;

    pDst = osal_mem_alloc(len);
    if (pDst)
    {
        osal_memcpy(pDst, src, len);
    }

    return ((void *)pDst);
}

/*********************************************************************
 * @fn osal_memcmp
 *
 * @brief
 *
 *   通用内存比较函数。
 *
 * @param   src1 - 源地址1
 * @param   src2 - 源地址2
 * @param   len - 要比较的字节数
 *
 * @return  TRUE - 内容相同，FALSE - 内容不同
 */
uint8 osal_memcmp(const void *src1, const void *src2, unsigned int len)
{
    const uint8 *pSrc1;
    const uint8 *pSrc2;

    pSrc1 = src1;
    pSrc2 = src2;

    while (len--)
    {
        if (*pSrc1++ != *pSrc2++)
            return FALSE;
    }
    return TRUE;
}

/*********************************************************************
 * @fn osal_memset
 *
 * @brief
 *
 *   将内存缓冲区设置为指定值。
 *
 * @param   dest - 缓冲区指针
 * @param   value - 设置每个字节为何值
 * @param   size - 大小
 *
 * @return  指向目标缓冲区的指针
 */
void *osal_memset(void *dest, uint8 value, int len)
{
    return memset(dest, value, len);
}