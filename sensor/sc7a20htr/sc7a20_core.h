/**
 ******************************************************************************
 * @file    SC7A20_core.h
 * @brief   SC7A20HTR 核心驱动接口
 ******************************************************************************
 * @note    本文件内定义传感器的核心驱动接口，采用静态内存分配
 *
 ******************************************************************************
 */
#ifndef SC7A20_CORE_H
#define SC7A20_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sc7a20_def.h"
#include "sc7a20_reg.h"
#include "sc7a20_hal.h"

/* ========================== 设备句柄定义 ========================== */

/**
 * @brief 设备句柄类型（避免动态内存分配）
 * 使用索引或指针，这里使用指针但指向静态分配的内存
 */

/**
 * @brief 设备上下文结构
 * @note 用户需要静态分配此结构体，避免动态内存分配
 */
typedef struct {
    sc7a20_ops_t ops;            // 硬件操作接口
    sc7a20_config_t config;      // 当前配置
    sc7a20_status_t last_status; // 最后操作状态

    // 设备状态标志
    struct
    {
        uint8_t is_initialized : 1; // 初始化标志
        uint8_t is_sleeping : 1;    // 睡眠状态标志
        uint8_t data_ready : 1;     // 数据就绪标志
        uint8_t reserved : 5;
    } flags;

    uint8_t chip_id;   // 芯片ID缓存
    float sensitivity; // 当前灵敏度系数(mg/LSB)

    // 统计信息（可选）
    uint32_t read_count;  // 读取次数统计
    uint32_t error_count; // 错误次数统计
} sc7a20_dev_t;
/* ========================== 设备管理接口 ========================== */
typedef sc7a20_dev_t *sc7a20_handle_t;
/**
 * @brief 初始化SC7A20传感器
 * @param handle 设备句柄指针（输出参数）
 * @param ops 硬件操作接口
 * @param config 设备配置
 * @return 操作状态
 * @note 使用静态内存分配，需提前分配sc7a20_dev_t结构体
 */
sc7a20_status_t sc7a20_init(sc7a20_handle_t handle, const sc7a20_ops_t *ops,
                            const sc7a20_config_t *config);

/**
 * @brief 反初始化传感器，释放资源
 * @param handle 设备句柄
 * @return 操作状态
 */
sc7a20_status_t sc7a20_deinit(sc7a20_handle_t handle);

/**
 * @brief 软件复位传感器
 * @param handle 设备句柄
 * @return 操作状态
 */
sc7a20_status_t sc7a20_reset(sc7a20_handle_t handle);

/**
 * @brief 检查设备是否就绪
 * @param handle 设备句柄
 * @param ready 就绪状态输出
 * @return 操作状态
 */
sc7a20_status_t sc7a20_is_ready(sc7a20_handle_t handle, bool *ready);

/* ========================== 数据采集接口 ========================== */

/**
 * @brief 读取加速度数据（包含原始数据和g值）
 * @param handle 设备句柄
 * @param data 加速度数据输出
 * @return 操作状态
 */
sc7a20_status_t sc7a20_read_acceleration(sc7a20_handle_t handle, sc7a20_accel_data_t *data);

/**
 * @brief 读取原始加速度数据（仅原始值）
 * @param handle 设备句柄
 * @param x X轴原始数据输出
 * @param y Y轴原始数据输出
 * @param z Z轴原始数据输出
 * @return 操作状态
 */
sc7a20_status_t sc7a20_read_raw_data(sc7a20_handle_t handle, int16_t *x, int16_t *y, int16_t *z);

/**
 * @brief 检查新数据是否就绪
 * @param handle 设备句柄
 * @param ready 数据就绪状态输出
 * @return 操作状态
 */
sc7a20_status_t sc7a20_is_data_ready(sc7a20_handle_t handle, bool *ready);

/**
 * @brief 从实时数据寄存器读取数据
 * @param handle 设备句柄
 * @param data 加速度数据输出
 * @return 操作状态
 */
sc7a20_status_t sc7a20_read_new_data(sc7a20_handle_t handle, sc7a20_accel_data_t *data);

/* ========================== 配置管理接口 ========================== */

/**
 * @brief 设置加速度计量程
 * @param handle 设备句柄
 * @param range 量程枚举值
 * @return 操作状态
 */
sc7a20_status_t sc7a20_set_range(sc7a20_handle_t handle, sc7a20_accel_fs_t range);

/**
 * @brief 设置输出数据率
 * @param handle 设备句柄
 * @param odr 数据率枚举值
 * @return 操作状态
 */
sc7a20_status_t sc7a20_set_output_data_rate(sc7a20_handle_t handle, sc7a20_accel_odr_t odr);

/**
 * @brief 设置轴使能状态
 * @param handle 设备句柄
 * @param x_enable X轴使能
 * @param y_enable Y轴使能
 * @param z_enable Z轴使能
 * @return 操作状态
 */
sc7a20_status_t sc7a20_set_axis_enable(sc7a20_handle_t handle, bool x_enable, bool y_enable, bool z_enable);

/**
 * @brief 获取当前配置
 * @param handle 设备句柄
 * @param config 配置输出
 * @return 操作状态
 */
sc7a20_status_t sc7a20_get_config(sc7a20_handle_t handle, sc7a20_config_t *config);

/* ========================== 电源管理接口 ========================== */

/**
 * @brief 进入睡眠模式
 * @param handle 设备句柄
 * @return 操作状态
 */
sc7a20_status_t sc7a20_enter_sleep(sc7a20_handle_t handle);

/**
 * @brief 从睡眠模式唤醒
 * @param handle 设备句柄
 * @return 操作状态
 */
sc7a20_status_t sc7a20_wakeup(sc7a20_handle_t handle);

/**
 * @brief 设置电源模式
 * @param handle 设备句柄
 * @param mode 电源模式
 * @return 操作状态
 */
sc7a20_status_t sc7a20_set_power_mode(sc7a20_handle_t handle, sc7a20_power_mode_t mode);

/* ========================== 设备信息接口 ========================== */

/**
 * @brief 读取芯片ID
 * @param handle 设备句柄
 * @param chip_id 芯片ID输出
 * @return 操作状态
 */
sc7a20_status_t sc7a20_get_chip_id(sc7a20_handle_t handle, uint8_t *chip_id);

/**
 * @brief 读取设备状态寄存器
 * @param handle 设备句柄
 * @param status 状态值输出
 * @return 操作状态
 */
sc7a20_status_t sc7a20_get_status(sc7a20_handle_t handle, uint8_t *status);

/* ========================== 预设配置接口 ========================== */

/**
 * @brief 应用高性能预设配置
 * @param handle 设备句柄
 * @return 操作状态
 */
sc7a20_status_t sc7a20_config_high_performance(sc7a20_handle_t handle);

/**
 * @brief 应用低功耗预设配置
 * @param handle 设备句柄
 * @return 操作状态
 */
sc7a20_status_t sc7a20_config_low_power(sc7a20_handle_t handle);

/* ========================== 简化接口（常用功能） ========================== */

/**
 * @brief 快速初始化并配置为默认模式
 * @param handle 设备句柄
 * @param ops 硬件操作接口
 * @param i2c_addr I2C地址
 * @return 操作状态
 */
sc7a20_status_t sc7a20_quick_init(sc7a20_handle_t handle, const sc7a20_ops_t *ops, uint8_t i2c_addr);

#ifdef __cplusplus
}
#endif

#endif /* SC7A20_CORE_H */