#include "osal_timer.h"
#include "osal_memory.h"
#include "osal_event.h"
#include "type.h"

typedef struct
{
    void *next;
    uint16 timeout;       // 定时时间，每过一个系统时钟会自减
    uint16 event_flag;    // 定时事件，定时时间减完产生任务事件
    uint8 task_id;        // 响应的任务ID
    uint16 reloadTimeout; // 重装定时时间
} osalTimerRec_t;         // 任务定时器，链表结构

/*********************************************************************
 * GLOBAL VARIABLES
 */
osalTimerRec_t *timerHead; // 任务定时器链表头指针
uint8 tmr_decr_time;       // 任务定时器更新时自减的数值单位
uint8 timerActive;         // 标识硬件定时器是否运行

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint32 osal_systemClock; // 记录系统时钟

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */
osalTimerRec_t *osalAddTimer(uint8 task_id, uint16 event_flag, uint16 timeout);
osalTimerRec_t *osalFindTimer(uint8 task_id, uint16 event_flag);
void osalDeleteTimer(osalTimerRec_t *rmTimer);
void osal_timer_activate(uint8 turn_on);
void osal_timer_hw_setup(uint8 turn_on);

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/
/*********************************************************************
 * @fn osalTimerInit
 *
 * @brief   初始化OSAL定时器系统。
 *
 * @param   none
 *
 * @return  none
 */
void osalTimerInit(void)
{
    OSAL_TIMER_TICKINIT(); // 初始化硬件定时器

    // 初始化递减时间单位
    tmr_decr_time = TIMER_DECR_TIME;

    // 初始化系统定时器
    osal_timer_activate(FALSE);
    timerActive = FALSE;

    osal_systemClock = 0;
}

/*********************************************************************
 * @fn osalAddTimer
 *
 * @brief   向定时器链表中添加一个定时器。
 *          调用此函数前必须关闭中断。
 *
 * @param   task_id      任务ID
 * @param   event_flag   事件标志
 * @param   timeout      超时时间(毫秒)
 *
 * @return  osalTimerRec_t * - 指向新创建定时器的指针
 */
osalTimerRec_t *osalAddTimer(uint8 task_id, uint16 event_flag, uint16 timeout)
{
    osalTimerRec_t *newTimer;
    osalTimerRec_t *srchTimer;

    // 先查找是否已存在相同定时器
    newTimer = osalFindTimer(task_id, event_flag);
    if (newTimer)
    {
        // 如果找到定时器，则更新其超时时间
        newTimer->timeout = timeout;

        return (newTimer);
    }
    else
    {
        // 创建新的定时器
        newTimer = osal_mem_alloc(sizeof(osalTimerRec_t));

        if (newTimer)
        {
            // 填充新定时器信息
            newTimer->task_id = task_id;
            newTimer->event_flag = event_flag;
            newTimer->timeout = timeout;
            newTimer->next = (void *)NULL;
            newTimer->reloadTimeout = 0;

            // 判断定时器链表是否为空
            if (timerHead == NULL)
            {
                // 设置为链表头
                timerHead = newTimer;
            }
            else
            {
                // 添加到链表末尾
                srchTimer = timerHead;

                // 找到最后一个节点
                while (srchTimer->next)
                    srchTimer = srchTimer->next;

                // 添加到链表中
                srchTimer->next = newTimer;
            }
            return (newTimer);
        }
        else
            return ((osalTimerRec_t *)NULL);
    }
}

/*********************************************************************
 * @fn osalFindTimer
 *
 * @brief   在定时器链表中查找指定定时器。
 *          调用此函数前必须关闭中断。
 *
 * @param   task_id      任务ID
 * @param   event_flag   事件标志
 *
 * @return  osalTimerRec_t *
 */
osalTimerRec_t *osalFindTimer(uint8 task_id, uint16 event_flag)
{
    osalTimerRec_t *srchTimer;

    // 从链表头开始查找
    srchTimer = timerHead;

    // 查找直到找到或到达链表末尾
    while (srchTimer)
    {
        if (srchTimer->event_flag == event_flag &&
            srchTimer->task_id == task_id)
            break;

        // 继续查找下一个
        srchTimer = srchTimer->next;
    }

    return (srchTimer);
}

/*********************************************************************
 * @fn osalDeleteTimer
 *
 * @brief   从定时器链表中删除指定定时器。
 *
 * @param   rmTimer  要删除的定时器指针
 *
 * @return  none
 */
void osalDeleteTimer(osalTimerRec_t *rmTimer)
{
    // 检查定时器是否存在
    if (rmTimer)
    {
        // 清除事件标志，osalTimerUpdate()会从链表中删除该定时器
        rmTimer->event_flag = 0;
    }
}

/*********************************************************************
 * @fn osal_timer_activate
 *
 * @brief
 *
 *   开启或关闭硬件定时器
 *
 * @param  uint8 turn_on - FALSE - 关闭, TRUE - 开启
 *
 * @return  none
 */
void osal_timer_activate(uint8 turn_on)
{
    osal_timer_hw_setup(turn_on);
    timerActive = turn_on;
}

/*********************************************************************
 * @fn osal_timer_hw_setup
 *
 * @brief
 *
 *   配置定时器硬件。
 *
 * @param  uint8 turn_on  开关标志
 *
 * @return  void
 */
void osal_timer_hw_setup(uint8 turn_on)
{
    if (turn_on)
    {
        OSAL_TIMER_TICKSTART();
    }
    else
    {
        OSAL_TIMER_TICKSTOP();
    }
}

/*********************************************************************
 * @fn osal_start_timerEx
 *
 * @brief
 *
 *   启动一个在n毫秒后到期的定时器。
 *   当定时器到期时，调用任务将收到指定事件。
 *
 * @param   uint8 taskID          要设置定时器的任务ID
 * @param   uint16 event_id       要通知的事件
 * @param   UNINT16 timeout_value 超时时间(毫秒)
 *
 * @return  SUCCESS, 或 NO_TIMER_AVAIL
 */
uint8 osal_start_timerEx(uint8 taskID, uint16 event_id, uint16 timeout_value)
{
    osalTimerRec_t *newTimer;

    HAL_ENTER_CRITICAL_SECTION(); // 关闭中断

    // 添加定时器
    newTimer = osalAddTimer(taskID, event_id, timeout_value);

    if (newTimer)
    {
        // 是否需要启动硬件定时器?
        if (timerActive == FALSE)
        {
            osal_timer_activate(TRUE);
        }
    }

    HAL_EXIT_CRITICAL_SECTION(); // 重新开启中断

    return ((newTimer != NULL) ? SUCCESS : NO_TIMER_AVAIL);
}

/*********************************************************************
 * @fn osal_start_reload_timer
 *
 * @brief
 *
 *   启动一个在n毫秒后到期的可重载定时器。
 *   当定时器到期时，调用任务将收到指定事件，并且定时器会重新加载超时值。
 *
 * @param   uint8 taskID          要设置定时器的任务ID
 * @param   uint16 event_id       要通知的事件
 * @param   UNINT16 timeout_value 超时时间(毫秒)
 *
 * @return  SUCCESS, 或 NO_TIMER_AVAIL
 */
uint8 osal_start_reload_timer(uint8 taskID, uint16 event_id, uint16 timeout_value)
{
    osalTimerRec_t *newTimer;

    HAL_ENTER_CRITICAL_SECTION(); // 关闭中断

    // 添加定时器
    newTimer = osalAddTimer(taskID, event_id, timeout_value);
    if (newTimer)
    {
        // 设置重载超时值
        newTimer->reloadTimeout = timeout_value;
    }

    if (newTimer)
    {
        // 是否需要启动硬件定时器?
        if (timerActive == FALSE)
        {
            osal_timer_activate(TRUE);
        }
    }

    HAL_EXIT_CRITICAL_SECTION(); // 重新开启中断

    return ((newTimer != NULL) ? SUCCESS : NO_TIMER_AVAIL);
}

/*********************************************************************
 * @fn osal_stop_timerEx
 *
 * @brief
 *
 *   停止已经启动的定时器。
 *   如果成功，函数将取消定时器并防止与定时器关联的事件被设置给调用任务。
 *
 * @param   uint8 task_id    要停止的定时器的任务ID
 * @param   uint16 event_id  要停止的定时器标识符
 *
 * @return  SUCCESS 或 INVALID_EVENT_ID
 */
uint8 osal_stop_timerEx(uint8 task_id, uint16 event_id)
{
    osalTimerRec_t *foundTimer;

    HAL_ENTER_CRITICAL_SECTION(); // 关闭中断

    // 查找要停止的定时器
    foundTimer = osalFindTimer(task_id, event_id);
    if (foundTimer)
    {
        osalDeleteTimer(foundTimer);
    }

    HAL_EXIT_CRITICAL_SECTION(); // 重新开启中断

    return ((foundTimer != NULL) ? SUCCESS : INVALID_EVENT_ID);
}

/*********************************************************************
 * @fn osal_get_timeoutEx
 *
 * @brief
 *
 *   获取指定定时器的剩余时间。
 *
 * @param   uint8 task_id    要检查的定时器任务ID
 * @param   uint16 event_id  要检查的定时器标识符
 *
 * @return  如果找到定时器则返回其剩余时间，否则返回0
 */
uint16 osal_get_timeoutEx(uint8 task_id, uint16 event_id)
{
    uint16 rtrn = 0;
    osalTimerRec_t *tmr;

    HAL_ENTER_CRITICAL_SECTION(); // 关闭中断

    tmr = osalFindTimer(task_id, event_id);

    if (tmr)
    {
        rtrn = tmr->timeout;
    }

    HAL_EXIT_CRITICAL_SECTION(); // 重新开启中断

    return rtrn;
}

/*********************************************************************
 * @fn osal_timer_num_active
 *
 * @brief
 *
 *   计算活动定时器的数量。
 *
 * @return  uint8 - 定时器数量
 */
uint8 osal_timer_num_active(void)
{
    uint8 num_timers = 0;
    osalTimerRec_t *srchTimer;

    HAL_ENTER_CRITICAL_SECTION(); // 关闭中断

    // 从链表头开始
    srchTimer = timerHead;

    // 计算链表中的定时器数量
    while (srchTimer != NULL)
    {
        num_timers++;
        srchTimer = srchTimer->next;
    }

    HAL_EXIT_CRITICAL_SECTION(); // 重新开启中断

    return num_timers;
}

/*********************************************************************
 * @fn osalTimerUpdate
 *
 * @brief   更新定时器结构以响应定时器滴答。
 *
 * @param   updateTime  更新时间单位
 *
 * @return  none
 *********************************************************************/
void osalTimerUpdate(uint16 updateTime)
{
    osalTimerRec_t *srchTimer;
    osalTimerRec_t *prevTimer;

    HAL_ENTER_CRITICAL_SECTION(); // 关闭中断
    // 更新系统时间
    osal_systemClock += updateTime;
    HAL_EXIT_CRITICAL_SECTION(); // 重新开启中断

    // 检查定时器链表是否存在
    if (timerHead != NULL)
    {
        // 从链表头开始遍历
        srchTimer = timerHead;
        prevTimer = (void *)NULL;

        // 遍历所有定时器
        while (srchTimer)
        {
            osalTimerRec_t *freeTimer = NULL;

            HAL_ENTER_CRITICAL_SECTION(); // 关闭中断

            if (srchTimer->timeout <= updateTime)
            {
                srchTimer->timeout = 0;
            }
            else
            {
                srchTimer->timeout = srchTimer->timeout - updateTime;
            }

            // 检查是否需要重载
            if ((srchTimer->timeout == 0) && (srchTimer->reloadTimeout) && (srchTimer->event_flag))
            {
                // 通知任务超时
                osal_set_event(srchTimer->task_id, srchTimer->event_flag);

                // 重载定时器超时值
                srchTimer->timeout = srchTimer->reloadTimeout;
            }

            // 当超时或被删除(event_flag == 0)
            if (srchTimer->timeout == 0 || srchTimer->event_flag == 0)
            {
                // 从链表中移除
                if (prevTimer == NULL)
                    timerHead = srchTimer->next;
                else
                    prevTimer->next = srchTimer->next;

                // 准备释放内存
                freeTimer = srchTimer;

                // 移动到下一个
                srchTimer = srchTimer->next;
            }
            else
            {
                // 继续下一个
                prevTimer = srchTimer;
                srchTimer = srchTimer->next;
            }

            HAL_EXIT_CRITICAL_SECTION(); // 重新开启中断

            if (freeTimer)
            {
                if (freeTimer->timeout == 0)
                {
                    osal_set_event(freeTimer->task_id, freeTimer->event_flag);
                }
                osal_mem_free(freeTimer);
            }
        }
    }
}

/*********************************************************************
 * @fn osal_GetSystemClock()
 *
 * @brief   读取本地系统时钟。
 *
 * @param   none
 *
 * @return  本地时钟值(毫秒)
 */
uint32 osal_GetSystemClock(void)
{
    return (osal_systemClock);
}

/*********************************************************************
 * @fn osal_update_timers
 *
 * @brief   更新定时器结构以响应定时器滴答。
 *
 * @param   none
 *
 * @return  none
 */
// 在每次系统定时溢出时调用一次，更新计时器
void osal_update_timers(void)
{
    osalTimerUpdate(tmr_decr_time);
}