/**
 * @file alarm_task.h
 * @brief 串口警报/切图命令入口。
 *
 * USART1 @115200，命令体固定为 8 字节 ASCII，末尾接受 LF、CR 或 CRLF：
 *   cmd:ARDA  空袭警报（鸣 6s 停 6s ×15）
 *   cmd:PREA  预先警报（鸣 36s 停 24s ×3）
 *   cmd:ACLR  解除警报（连续鸣 180s）
 *   cmd:STOP  任意模式立即停
 *   cmd:IMG1 / cmd:IMG2 / cmd:IMG3  请求切图
 */
#ifndef APP_ALARM_TASK_H
#define APP_ALARM_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void alarm_task_register(void);

/** 1=正在警报，heartbeat 应让出蜂鸣器 */
uint8_t alarm_is_active(void);

/** 串口请求的图号 1..3；lcd 可轮询 */
uint8_t alarm_pending_image(void);
void alarm_clear_pending_image(void);

/** USB CDC 等通道灌入命令字节（与 USART1 同一解析） */
void alarm_feed_byte(uint8_t b);

#ifdef __cplusplus
}
#endif

#endif
