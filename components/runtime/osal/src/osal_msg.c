#include "osal_msg.h"
#include "osal_memory.h"
#include "osal_event.h"

#define SYS_EVENT_MSG 0x8000

/*********************************************************************
 * @fn osal_msg_allocate
 *
 * @brief
 *
 *    此函数由任务调用，用于分配一个消息缓冲区，
 *    任务将在其中编码它希望发送的特定消息。这种通用缓冲区方案
 *    用于严格限制由于微处理器RAM大小限制而在系统内创建的消息缓冲区。
 *    注意所有消息缓冲区都是固定大小的（至少最初是这样）。
 *    参数len保留以备将来创建具有不同固定消息大小的消息池时使用
 *    （例如，可以维护和分配大、中、小三种尺寸的消息缓冲池）。
 *
 * @param   uint8 len  - 所需缓冲区长度
 * @return  指向已分配缓冲区的指针，如果分配失败则返回NULL
 */
uint8 *osal_msg_allocate(uint16 len)
{
    osal_msg_hdr_t *hdr;

    if (len == 0)
    {
        return (NULL);
    }

    hdr = (osal_msg_hdr_t *)osal_mem_alloc((short)(len + sizeof(osal_msg_hdr_t)));
    if (hdr)
    {
        hdr->next = NULL;
        hdr->len = len;
        hdr->dest_id = TASK_NO_TASK;
        return ((uint8 *)(hdr + 1));
    }
    else
    {
        return (NULL);
    }
}

/*********************************************************************
 * @fn osal_msg_deallocate
 *
 * @brief
 *
 *    此函数用于释放消息缓冲区。当任务（或处理元素）
 *    完成对接收消息的处理后，会调用此函数。
 *
 *
 * @param   uint8 *msg_ptr - 指向新消息缓冲区的指针
 *
 * @return  SUCCESS, INVALID_MSG_POINTER
 */
uint8 osal_msg_deallocate(uint8 *msg_ptr)
{
    uint8 *x;

    if (msg_ptr == NULL)
    {
        return (INVALID_MSG_POINTER);
    }

    // 不要释放排队中的缓冲区
    if (OSAL_MSG_ID(msg_ptr) != TASK_NO_TASK)
    {
        return (MSG_BUFFER_NOT_AVAIL);
    }

    x = (uint8 *)((uint8 *)msg_ptr - sizeof(osal_msg_hdr_t));

    osal_mem_free((void *)x);

    return (SUCCESS);
}

/*********************************************************************
 * @fn osal_msg_send
 *
 * @brief
 *
 *    此函数由任务调用，用于向另一个任务或处理元素发送命令消息。
 *    sending_task字段必须引用有效任务，因为任务ID将用于响应消息。
 *    此函数还将在目标任务的事件列表中设置消息就绪事件。
 *
 *
 * @param   uint8 destination_task - 发送消息给哪个任务？任务ID
 * @param   uint8 *msg_ptr - 指向新消息缓冲区的指针
 * @param   uint8 len - 消息中数据的长度
 *
 * @return  SUCCESS, INVALID_TASK, INVALID_MSG_POINTER
 */
uint8 osal_msg_send(uint8 destination_task, uint8 *msg_ptr)
{
    if (msg_ptr == NULL)
    {
        return (INVALID_MSG_POINTER);
    }

    if (destination_task >= tasksCnt)
    {
        osal_msg_deallocate(msg_ptr);
        return (INVALID_TASK);
    }

    // 检查消息头部
    if (OSAL_MSG_NEXT(msg_ptr) != NULL ||
        OSAL_MSG_ID(msg_ptr) != TASK_NO_TASK)
    {
        osal_msg_deallocate(msg_ptr);
        return (INVALID_MSG_POINTER);
    }

    OSAL_MSG_ID(msg_ptr) = destination_task;

    // 将消息入队
    osal_msg_enqueue(&osal_qHead, msg_ptr);

    // 向目标任务发出有消息等待的信号
    osal_set_event(destination_task, SYS_EVENT_MSG);

    return (SUCCESS);
}

/*********************************************************************
 * @fn osal_msg_receive
 *
 * @brief
 *
 *    此函数由任务调用以检索接收到的命令消息。
 *    调用任务在使用osal_msg_deallocate()调用处理完消息后，
 *    必须释放消息缓冲区。
 *
 * @param   uint8 task_id - 接收任务的ID
 *
 * @return  *uint8 - 消息信息，如果没有消息则返回NULL
 */
uint8 *osal_msg_receive(uint8 task_id)
{
    osal_msg_hdr_t *listHdr;
    osal_msg_hdr_t *prevHdr = NULL;
    osal_msg_hdr_t *foundHdr = NULL;

    // 关闭中断
    HAL_ENTER_CRITICAL_SECTION();

    // 指向队列顶部
    listHdr = osal_qHead;

    // 遍历队列寻找属于请求任务的消息
    while (listHdr != NULL)
    {
        if ((listHdr - 1)->dest_id == task_id)
        {
            if (foundHdr == NULL)
            {
                // 保存第一个匹配的消息
                foundHdr = listHdr;
            }
            else
            {
                // 找到第二个消息，停止查找
                break;
            }
        }
        if (foundHdr == NULL)
        {
            prevHdr = listHdr;
        }
        listHdr = OSAL_MSG_NEXT(listHdr);
    }

    // 是否还有更多消息？
    if (listHdr != NULL)
    {
        // 是的，向任务发出有消息等待的信号
        osal_set_event(task_id, SYS_EVENT_MSG);
    }
    else
    {
        // 没有更多消息
        osal_clear_event(task_id, SYS_EVENT_MSG);
    }

    // 是否找到了消息？
    if (foundHdr != NULL)
    {
        // 从链表中取出
        osal_msg_extract(&osal_qHead, foundHdr, prevHdr);
    }

    // 恢复中断
    HAL_EXIT_CRITICAL_SECTION();

    return ((uint8 *)foundHdr);
}

/**************************************************************************************************
 * @fn     osal_msg_find
 *
 * @brief       此函数在队列中查找匹配task_id和event参数的OSAL消息。
 *
 * 输入参数
 *
 * @param       task_id - 入队的OSAL消息必须匹配的OSAL任务ID
 * @param       event - 入队的OSAL消息必须匹配的OSAL事件ID
 *
 * 输出参数
 *
 * 无
 *
 * @return      如果没有匹配项返回NULL，否则返回指向匹配的OSAL消息的原地指针
 **************************************************************************************************
 */
osal_event_hdr_t *osal_msg_find(uint8 task_id, uint8 event)
{
    osal_msg_hdr_t *pHdr;

    HAL_ENTER_CRITICAL_SECTION(); // 关闭中断

    pHdr = osal_qHead; // 指向队列顶部

    // 遍历队列查找匹配task_id和event参数的消息
    while (pHdr != NULL)
    {
        if (((pHdr - 1)->dest_id == task_id) && (((osal_event_hdr_t *)pHdr)->event == event))
        {
            break;
        }

        pHdr = OSAL_MSG_NEXT(pHdr);
    }

    HAL_EXIT_CRITICAL_SECTION(); // 恢复中断

    return (osal_event_hdr_t *)pHdr;
}

/*********************************************************************
 * @fn osal_msg_enqueue
 *
 * @brief
 *
 *    此函数将OSAL消息加入OSAL队列。
 *
 * @param   osal_msg_q_t *q_ptr - OSAL队列
 * @param   void *msg_ptr  - OSAL消息
 *
 * @return  none
 */
void osal_msg_enqueue(osal_msg_q_t *q_ptr, void *msg_ptr)
{
    void *list;

    // 关闭中断
    HAL_ENTER_CRITICAL_SECTION();

    OSAL_MSG_NEXT(msg_ptr) = NULL;
    // 如果是队列中的第一条消息
    if (*q_ptr == NULL)
    {
        *q_ptr = msg_ptr;
    }
    else
    {
        // 找到队列末尾
        for (list = *q_ptr; OSAL_MSG_NEXT(list) != NULL; list = OSAL_MSG_NEXT(list))
            ;

        // 将消息添加到队列末尾
        OSAL_MSG_NEXT(list) = msg_ptr;
    }

    // 重新启用中断
    HAL_EXIT_CRITICAL_SECTION();
}

/*********************************************************************
 * @fn osal_msg_dequeue
 *
 * @brief
 *
 *    此函数从OSAL队列中取出一个OSAL消息。
 *
 * @param   osal_msg_q_t *q_ptr - OSAL队列
 *
 * @return  void * - 指向OSAL消息的指针，如果队列为空则返回NULL
 */
void *osal_msg_dequeue(osal_msg_q_t *q_ptr)
{
    void *msg_ptr = NULL;

    // 关闭中断
    HAL_ENTER_CRITICAL_SECTION();

    if (*q_ptr != NULL)
    {
        // 取出消息
        msg_ptr = *q_ptr;
        *q_ptr = OSAL_MSG_NEXT(msg_ptr);
        OSAL_MSG_NEXT(msg_ptr) = NULL;
        OSAL_MSG_ID(msg_ptr) = TASK_NO_TASK;
    }

    // 重新启用中断
    HAL_EXIT_CRITICAL_SECTION();

    return msg_ptr;
}

/*********************************************************************
 * @fn osal_msg_push
 *
 * @brief
 *
 *    此函数将OSAL消息推入OSAL队列的头部。
 *
 * @param   osal_msg_q_t *q_ptr - OSAL队列
 * @param   void *msg_ptr  - OSAL消息
 *
 * @return  none
 */
void osal_msg_push(osal_msg_q_t *q_ptr, void *msg_ptr)
{
    // 关闭中断
    HAL_ENTER_CRITICAL_SECTION();

    // 将消息推入队列头部
    OSAL_MSG_NEXT(msg_ptr) = *q_ptr;
    *q_ptr = msg_ptr;

    // 重新启用中断
    HAL_EXIT_CRITICAL_SECTION();
}

/*********************************************************************
 * @fn osal_msg_extract
 *
 * @brief
 *
 *    此函数从OSAL队列中间提取并移除一个OSAL消息。
 *
 * @param   osal_msg_q_t *q_ptr - OSAL队列
 * @param   void *msg_ptr  - 要提取的OSAL消息
 * @param   void *prev_ptr  - 队列中位于msg_ptr之前的消息
 *
 * @return  none
 */
void osal_msg_extract(osal_msg_q_t *q_ptr, void *msg_ptr, void *prev_ptr)
{
    // 关闭中断
    HAL_ENTER_CRITICAL_SECTION();

    if (msg_ptr == *q_ptr)
    {
        // 从头部移除
        *q_ptr = OSAL_MSG_NEXT(msg_ptr);
    }
    else
    {
        // 从中间移除
        OSAL_MSG_NEXT(prev_ptr) = OSAL_MSG_NEXT(msg_ptr);
    }
    OSAL_MSG_NEXT(msg_ptr) = NULL;
    OSAL_MSG_ID(msg_ptr) = TASK_NO_TASK;

    // 重新启用中断
    HAL_EXIT_CRITICAL_SECTION();
}

/*********************************************************************
 * @fn osal_msg_enqueue_max
 *
 * @brief
 *
 *    此函数在队列长度小于最大值时将OSAL消息加入OSAL队列。
 *
 * @param   osal_msg_q_t *q_ptr - OSAL队列
 * @param   void *msg_ptr  - OSAL消息
 * @param   uint8 max - 队列的最大长度
 *
 * @return  如果消息成功入队返回TRUE，否则返回FALSE
 */
uint8 osal_msg_enqueue_max(osal_msg_q_t *q_ptr, void *msg_ptr, uint8 max)
{
    void *list;
    uint8 ret = FALSE;

    // 关闭中断
    HAL_ENTER_CRITICAL_SECTION();

    // 如果是队列中的第一条消息
    if (*q_ptr == NULL)
    {
        *q_ptr = msg_ptr;
        ret = TRUE;
    }
    else
    {
        // 找到队列末尾或达到最大值
        list = *q_ptr;
        max--;
        while ((OSAL_MSG_NEXT(list) != NULL) && (max > 0))
        {
            list = OSAL_MSG_NEXT(list);
            max--;
        }

        // 如果未达到最大值，则将消息添加到队列末尾
        if (max != 0)
        {
            OSAL_MSG_NEXT(list) = msg_ptr;
            ret = TRUE;
        }
    }

    // 重新启用中断
    HAL_EXIT_CRITICAL_SECTION();

    return ret;
}