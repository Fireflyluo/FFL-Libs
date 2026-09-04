#include "osal_event.h"
#include "osal_memory.h"

OsalTadkREC_t *TaskHead;
OsalTadkREC_t *TaskActive;

uint8 Task_id;  // 任务ID统计
uint8 tasksCnt; // 任务数量统计

/*********************************************************************
 * @fn osal_set_event
 *
 * @brief
 *
 *    此函数用于设置任务的事件标志。传入的事件将通过OR操作添加到任务的事件变量中。
 *
 * @param   byte task_id - 接收任务的ID
 * @param   byte event_flag - 要设置的事件
 *
 * @return  ZSUCCESS, INVALID_TASK
 */
uint8 osal_set_event(byte task_id, uint16 event_flag)
{
    OsalTadkREC_t *srchTask;

    srchTask = osalFindTask(task_id);
    if (srchTask)
    {
        // 关闭中断
        HAL_ENTER_CRITICAL_SECTION();
        // 设置事件位
        srchTask->events |= event_flag;
        // 恢复中断
        HAL_EXIT_CRITICAL_SECTION();
    }
    else
        return (INVALID_TASK);

    return (ZSUCCESS);
}

/*********************************************************************
 * @fn osal_clear_event
 *
 * @brief
 *
 *    此函数用于清除任务的事件标志。传入的事件将通过掩码操作从任务的事件变量中清除。
 *
 * @param   uint8 task_id - 接收任务的ID
 * @param   uint8 event_flag - 要清除的事件
 *
 * @return  SUCCESS, INVALID_TASK
 */
uint8 osal_clear_event(uint8 task_id, uint16 event_flag)
{
    OsalTadkREC_t *srchTask;

    srchTask = osalFindTask(task_id);
    if (srchTask)
    {
        // 关闭中断
        HAL_ENTER_CRITICAL_SECTION();
        // 清除事件位
        srchTask->events &= ~event_flag;
        // 恢复中断
        HAL_EXIT_CRITICAL_SECTION();
    }
    else
        return (INVALID_TASK);

    return (ZSUCCESS);
}

/***************************************************************************
 * @fn osal_init_TaskHead
 *
 * @brief   初始化任务链表头
 *
 * @param   none
 *
 * @return  none
 */
void osal_init_TaskHead(void)
{
    TaskHead = (OsalTadkREC_t *)NULL;
    TaskActive = (OsalTadkREC_t *)NULL;
    Task_id = 0;
}

/***************************************************************************
 * @fn osal_Task_init
 *
 * @brief   初始化任务
 *
 * @param   none
 *
 * @return  none
 */
void osal_Task_init(void)
{
    TaskActive = TaskHead;
    while (TaskActive)
    {
        if (TaskActive->pfnInit)
        {
            TaskActive->pfnInit(TaskActive->taskID);
        }
        TaskActive = TaskActive->next;
    }
    TaskActive = (OsalTadkREC_t *)NULL;
}

/***************************************************************************
 * @fn osal_add_Task
 *
 * @brief   添加任务到任务链表
 *
 * @param   pfnInit              - 任务初始化函数指针
 * @param   pfnEventProcessor    - 任务事件处理函数指针
 * @param   taskPriority         - 任务优先级
 *
 * @return  none
 */
void osal_add_Task(pTaskInitFn pfnInit,
                   pTaskEventHandlerFn pfnEventProcessor,
                   uint8 taskPriority)
{
    OsalTadkREC_t *TaskNew;
    OsalTadkREC_t *TaskSech;
    OsalTadkREC_t **TaskPTR;
    TaskNew = osal_mem_alloc(sizeof(OsalTadkREC_t));
    if (TaskNew)
    {
        TaskNew->pfnInit = pfnInit;
        TaskNew->pfnEventProcessor = pfnEventProcessor;
        TaskNew->taskID = Task_id++;
        TaskNew->events = 0;
        TaskNew->taskPriority = taskPriority;
        TaskNew->next = (OsalTadkREC_t *)NULL;

        TaskPTR = &TaskHead;
        TaskSech = TaskHead;

        tasksCnt++; // 任务数量统计

        while (TaskSech)
        {
            if (TaskNew->taskPriority > TaskSech->taskPriority)
            {
                TaskNew->next = TaskSech;
                *TaskPTR = TaskNew;
                return;
            }
            TaskPTR = &TaskSech->next;
            TaskSech = TaskSech->next;
        }
        *TaskPTR = TaskNew;
    }
    return;
}

/*********************************************************************
 * @fn osalNextActiveTask
 *
 * @brief   此函数将返回下一个活动任务。
 *
 * 注意:    任务队列按优先级排序。我们可以在第一个"就绪"任务（事件元素非零）处停止
 *
 * @param   none
 *
 * @return  指向找到任务的指针，如果未找到则返回NULL
 */
OsalTadkREC_t *osalNextActiveTask(void)
{
    OsalTadkREC_t *TaskSech;

    // 从开头开始
    TaskSech = TaskHead;

    // 查找直到找到或未找到
    while (TaskSech)
    {
        if (TaskSech->events)
        {
            // 任务是最高优先级且已就绪
            return TaskSech;
        }
        TaskSech = TaskSech->next;
    }
    return NULL;
}

/*********************************************************************
 * @fn osalFindActiveTask
 *
 * @brief   此函数将根据任务ID返回对应任务。
 *
 * 注意:    任务队列按优先级排序。我们可以在第一个"就绪"任务（事件元素非零）处停止
 *
 * @param   task_id  任务ID
 *
 * @return  指向找到任务的指针，如果未找到则返回NULL
 */
OsalTadkREC_t *osalFindTask(uint8 taskID)
{
    OsalTadkREC_t *TaskSech;
    TaskSech = TaskHead;
    while (TaskSech)
    {
        if (TaskSech->taskID == taskID)
        {
            return (TaskSech);
        }
        TaskSech = TaskSech->next;
    }
    return ((OsalTadkREC_t *)NULL);
}