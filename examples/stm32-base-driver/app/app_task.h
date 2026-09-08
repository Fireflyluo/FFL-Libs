/**
 * @file app_task.h
 * @brief 应用任务注册入口。
 */
#ifndef APP_TASK_H
#define APP_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/** 把应用任务注册进 ffl.osal（在 osal_Task_init() 之前调用）。 */
void app_task_register(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK_H */
