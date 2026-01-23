/**
 ******************************************************************************
 * @file    qmi8658a_driver.h
 * @brief   QMI8658A IMU传感器驱动头文件
 ******************************************************************************
 */

#ifndef QMI8658A_DRIVER_H
#define QMI8658A_DRIVER_H

#include "qmi8658a_reg.h"
#include "gyro_device.h"
#include "py32f4xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif

/* 设备上下文结构体扩展 */
typedef struct {
    qmi8658a_ctx_t reg_ctx;          // 寄存器操作上下文
    gyro_range_t current_gyro_range;  // 当前陀螺仪量程
    qmi8658a_accel_fs_t current_accel_fs; // 当前加速度计量程
} qmi8658a_driver_ctx_t;

/* 导出统一接口 */
extern const gyro_device_t qmi8658a_device;
int32_t qmi8658a_init(void *hardware_handle);
#ifdef __cplusplus
}
#endif

#endif /* QMI8658A_DRIVER_H */
