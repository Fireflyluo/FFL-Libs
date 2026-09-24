/**
 * @file app_task.h
 * @brief 把应用任务注册进 ffl.osal 的唯一入口。
 *
 * 调用时机：osal_init_system() 之后、osal_Task_init() 之前。
 * 实现见 app_task.c：LED 心跳、SC7A20 采样、ringbuffer 统计、蜂鸣器。
 */
#ifndef APP_TASK_H
#define APP_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/** osal_add_Task(...)：向 ffl.osal 注册本示例业务任务。 */
void app_task_register(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK_H */
