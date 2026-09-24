/**
 * @file osal_tasks.h
 * @brief 各外设/组件演示任务的注册入口（均由 main 在 osal 初始化后调用）。
 *
 * 拆分原则：每个 XXX_task.c 只演示一类组件用法，互不 include 对方业务头。
 */
#ifndef OSAL_TASKS_H
#define OSAL_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

/** Task：LED 心跳 + 蜂鸣器（OSAL reload + sw_timer 单次）。 */
void heartbeat_task_register(void);

/** Task：SC7A20 采样 + ringbuffer + 统计（驱动 + port）。 */
void sc7a20_task_register(void);

/** Task：W25Q **软 SPI** 初始化与探测（图像经 USB 写入）。 */
void flash_task_register(void);

/** Task：ST7735 硬 SPI1+DMA 刷屏，5s 换图（源=W25Q）。 */
void lcd_task_register(void);

/** Task：串口警报命令 + 蜂鸣器时序。 */
void alarm_task_register(void);

/** Task5：atomic/ringbuffer/sw_timer 功能点自检（上电一次）。 */
void features_demo_run(void);

#ifdef __cplusplus
}
#endif

#endif
