#ifndef __ICM42688P_H__
#define __ICM42688P_H__

#include <stdint.h>
#include <stdbool.h>

#include "icm42688_reg.h"

#ifdef __cplusplus
extern "C"
{
#endif
/* ========================================================================== */
/*                          常量定义                                         */
/* ========================================================================== */

/* 设备识别ID */
#define ICM42688_WHO_AM_I_ID 0x47

/* 默认配置参数 */
#define ICM42688_DEFAULT_GYRO_FS ICM42688_GYRO_FS_1000DPS
#define ICM42688_DEFAULT_ACCEL_FS ICM42688_ACCEL_FS_8G
#define ICM42688_DEFAULT_ODR ICM42688_ODR_1000HZ
#define ICM42688_DEFAULT_FIFO_WATERMARK 512

/* 灵敏度缩放因子（LSB/物理单位） */
#define ICM42688_GYRO_SENSITIVITY_2000DPS 16.384f // LSB/dps
#define ICM42688_GYRO_SENSITIVITY_1000DPS 32.768f
#define ICM42688_GYRO_SENSITIVITY_500DPS 65.536f
#define ICM42688_GYRO_SENSITIVITY_250DPS 131.072f
#define ICM42688_GYRO_SENSITIVITY_125DPS 262.144f
#define ICM42688_GYRO_SENSITIVITY_62_5DPS 524.288f
#define ICM42688_GYRO_SENSITIVITY_31_25DPS 1048.576f
#define ICM42688_GYRO_SENSITIVITY_15_625DPS 2097.152f

#define ICM42688_ACCEL_SENSITIVITY_16G 2048.0f // LSB/g
#define ICM42688_ACCEL_SENSITIVITY_8G 4096.0f
#define ICM42688_ACCEL_SENSITIVITY_4G 8192.0f
#define ICM42688_ACCEL_SENSITIVITY_2G 16384.0f

/* 温度转换系数 */
#define ICM42688_TEMP_SENSITIVITY 132.48f // LSB/°C
#define ICM42688_TEMP_OFFSET 25.0f        // °C

/* 物理单位转换系数 */
#define ICM42688_G_TO_M_S2 9.80665f      // g to m/s²
#define ICM42688_DPS_TO_RAD_S 0.0174533f // dps to rad/s

/* 超时设置 */
#define ICM42688_DEFAULT_TIMEOUT_MS 1000
#define ICM42688_RESET_DELAY_MS 1000
#define ICM42688_MODE_SWITCH_DELAY_MS 100
#define ICM42688_WAKEUP_DELAY_MS 10

/* FIFO相关常量 */
#define ICM42688_FIFO_SIZE 2048           // 字节
#define ICM42688_FIFO_MAX_PACKETS 170     // 最大数据包数（基于12字节包）
#define ICM42688_FIFO_HEADER_SIZE 1       // 包头大小
#define ICM42688_FIFO_PACKET_BASE_SIZE 12 // 基础数据包大小

/* 中断源掩码 */
#define ICM42688_INT_SOURCE_DATA_READY (1 << 0)
#define ICM42688_INT_SOURCE_FIFO_THS (1 << 1)
#define ICM42688_INT_SOURCE_FIFO_OVF (1 << 2)
#define ICM42688_INT_SOURCE_APEX (1 << 3)

/* APEX功能掩码 */
#define ICM42688_APEX_PEDOMETER_MASK (1 << 0)
#define ICM42688_APEX_TILT_DETECTION_MASK (1 << 1)
#define ICM42688_APEX_RAISE_TO_WAKE_MASK (1 << 2)
#define ICM42688_APEX_TAP_DETECTION_MASK (1 << 3)
#define ICM42688_APEX_WAKE_ON_MOTION_MASK (1 << 4)
#define ICM42688_APEX_SIGNIFICANT_MOTION_MASK (1 << 5)

/* 基于手册典型值的阈值建议（物理量单位） */
#define ICM42688_GYRO_SELF_TEST_TYPICAL 200.0f  // 典型值 ±200 dps
#define ICM42688_GYRO_SELF_TEST_TOLERANCE 20.0f // 容忍范围 ±20 dps
#define ICM42688_GYRO_SELF_TEST_MIN (ICM42688_GYRO_SELF_TEST_TYPICAL - ICM42688_GYRO_SELF_TEST_TOLERANCE)
#define ICM42688_GYRO_SELF_TEST_MAX (ICM42688_GYRO_SELF_TEST_TYPICAL + ICM42688_GYRO_SELF_TEST_TOLERANCE)

#define ICM42688_ACCEL_SELF_TEST_TYPICAL 200.0f  // 典型值 ±200 mg
#define ICM42688_ACCEL_SELF_TEST_TOLERANCE 20.0f // 容忍范围 ±20 mg
#define ICM42688_ACCEL_SELF_TEST_MIN (ICM42688_ACCEL_SELF_TEST_TYPICAL - ICM42688_ACCEL_SELF_TEST_TOLERANCE)
#define ICM42688_ACCEL_SELF_TEST_MAX (ICM42688_ACCEL_SELF_TEST_TYPICAL + ICM42688_ACCEL_SELF_TEST_TOLERANCE)

    /* ========================================================================== */
    /*                          错误码定义                                       */
    /* ========================================================================== */
    /**
     * @brief 驱动错误码
     */
    typedef enum
    {
        ICM42688_OK = 0,                   // 操作成功
        ICM42688_ERR_COMM = -1,            // 通信错误
        ICM42688_ERR_NOT_READY = -2,       // 设备未就绪
        ICM42688_ERR_INVALID_PARAM = -3,   // 无效参数
        ICM42688_ERR_TIMEOUT = -4,         // 操作超时
        ICM42688_ERR_FIFO_OVERFLOW = -5,   // FIFO溢出
        ICM42688_ERR_SELF_TEST_FAIL = -6,  // 自检失败
        ICM42688_ERR_NOT_INITIALIZED = -7, // 设备未初始化
    } icm42688_err_t;

    /* ========================================================================== */
    /*                          函数指针类型定义                                 */
    /* ========================================================================== */
    /**
     * @brief 寄存器写函数指针类型
     * @param dev_addr 设备地址（I2C地址或SPI片选）
     * @param reg_addr 寄存器地址
     * @param data 要写入的数据缓冲区
     * @param len 数据长度
     * @return 错误码
     */
    typedef icm42688_err_t (*icm42688_write_reg_fn)(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len);

    /**
     * @brief 寄存器读函数指针类型
     * @param dev_addr 设备地址（I2C地址或SPI片选）
     * @param reg_addr 寄存器地址
     * @param data 读取数据的缓冲区
     * @param len 数据长度
     * @return 错误码
     */
    typedef icm42688_err_t (*icm42688_read_reg_fn)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

    /**
     * @brief 延时函数指针类型
     * @param ms 延时时间（毫秒）
     */
    typedef void (*icm42688_delay_ms_fn)(uint32_t ms);

    /**
     * @brief 获取系统 tick 函数指针类型
     * @return 当前系统 tick 值（单位：毫秒）
     */
    typedef uint32_t (*icm42688_get_tick_fn)(void);

    /**
     * @brief 中断回调函数指针类型
     * @param int_num 中断号（0: INT1, 1: INT2）
     * @param user_data 用户数据
     */
    typedef void (*icm42688_int_callback_fn)(uint8_t int_num, void *user_data);

    /* ========================================================================== */
    /*                          通信接口配置结构体                               */
    /* ========================================================================== */

    /**
     * @brief 通信接口类型
     */
    typedef enum
    {
        ICM42688_INTERFACE_SPI = 0, // SPI接口
        ICM42688_INTERFACE_I2C = 1, // I2C接口
        ICM42688_INTERFACE_I3C = 2, // I3C接口
    } icm42688_interface_t;

    /**
     * @brief 通信接口配置结构体
     */
    typedef struct
    {
        icm42688_interface_t interface;  // 通信接口类型
        uint8_t dev_addr;                // 设备地址
        icm42688_write_reg_fn write_reg; // 寄存器写函数
        icm42688_read_reg_fn read_reg;   // 寄存器读函数
        icm42688_delay_ms_fn delay_ms;   // 延时函数
        icm42688_get_tick_fn get_tick;   // 获取系统 tick 函数（可选）
    } icm42688_comm_config_t;
    /* ========================================================================== */
    /*                          传感器配置结构体                                 */
    /* ========================================================================== */
    /**
     * @brief 传感器配置结构体
     */
    typedef struct
    {
        /* 陀螺仪配置 */
        icm42688_sensor_mode_t gyro_mode;          // 陀螺仪工作模式
        icm42688_gyro_fs_t gyro_fs;                // 陀螺仪满量程范围
        icm42688_odr_t gyro_odr;                   // 陀螺仪输出数据速率
        icm42688_filter_order_t gyro_filter_order; // 陀螺仪滤波器阶数

        /* 加速度计配置 */
        icm42688_sensor_mode_t accel_mode;          // 加速度计工作模式
        icm42688_accel_fs_t accel_fs;               // 加速度计满量程范围
        icm42688_odr_t accel_odr;                   // 加速度计输出数据速率
        icm42688_filter_order_t accel_filter_order; // 加速度计滤波器阶数

        /* FIFO配置 */
        bool fifo_accel_en;      // 加速度计FIFO使能
        bool fifo_gyro_en;       // 陀螺仪FIFO使能
        bool fifo_temp_en;       // 温度FIFO使能
        bool fifo_tmst_fsync_ev; // 时间戳/FSYNC 数据包发送⾄ FIFO
        uint16_t fifo_watermark; // FIFO水印值

        /* 高级功能启用标志配置 */
        bool apex_en;      // APEX功能使能
        bool fifo_en;      // FIFO使能
        bool interrupt_en; // 中断使能

    } icm42688_sensor_config_t;
    /* 中断配置 */
    typedef struct
    {
        uint8_t int_pin;                       // 中断引脚选择（1: INT1, 2: INT2）
        uint8_t int_config;                    // 中断配置位掩码
        icm42688_int_mode_t int1_mode;         // INT1引脚模式
        icm42688_int_drive_t int1_drive;       // INT1驱动方式
        icm42688_int_polarity_t int1_polarity; // INT1极性
        icm42688_int_mode_t int2_mode;         // INT2引脚模式
        icm42688_int_drive_t int2_drive;       // INT2驱动方式
        icm42688_int_polarity_t int2_polarity; // INT2极性
    } icm42688_int_config_t;
    /* ========================================================================== */
    /*                          传感器数据输出结构体                                 */
    /* ========================================================================== */

    /**
     * @brief 原始传感器数据（16位）
     */
    typedef struct
    {
        int16_t accel_x;     // X轴加速度原始数据
        int16_t accel_y;     // Y轴加速度原始数据
        int16_t accel_z;     // Z轴加速度原始数据
        int16_t gyro_x;      // X轴陀螺仪原始数据
        int16_t gyro_y;      // Y轴陀螺仪原始数据
        int16_t gyro_z;      // Z轴陀螺仪原始数据
        int16_t temperature; // 温度原始数据
    } icm42688_raw_data_t;

    /**
     * @brief 转换后的传感器数据（物理量）
     */
    typedef struct
    {
        float accel_x;      // X轴加速度 [m/s²]
        float accel_y;      // Y轴加速度 [m/s²]
        float accel_z;      // Z轴加速度 [m/s²]
        float gyro_x;       // X轴角速度 [rad/s]
        float gyro_y;       // Y轴角速度 [rad/s]
        float gyro_z;       // Z轴角速度 [rad/s]
        float temperature;  // 温度 [°C]
        uint32_t timestamp; // 时间戳 [ms]
    } icm42688_sensor_data_t;

    /**
     * @brief FIFO数据包结构
     */
    typedef struct
    {
        uint8_t header;      // FIFO包头
        int16_t accel[3];    // 加速度数据
        int16_t gyro[3];     // 陀螺仪数据
        uint8_t temperature; // 温度数据
        uint32_t timestamp;  // 时间戳
        uint8_t packet_size; // 数据包大小
    } icm42688_fifo_packet_t;

    /**
     * @brief 计步器数据
     */
    typedef struct
    {
        uint32_t step_count;    // 步数计数
        uint16_t step_detected; // 步数检测标志
        uint8_t step_cadence;   // 步频
        uint8_t activity_class; // 活动分类
    } icm42688_pedometer_data_t;

    /**
     * @brief 敲击检测数据
     */
    typedef struct
    {
        uint8_t tap_count;     // 敲击次数
        uint8_t tap_axis;      // 敲击轴
        uint8_t tap_direction; // 敲击方向
    } icm42688_tap_data_t;

    /**
     * @brief 设备状态信息
     */
    typedef struct
    {
        bool sensor_ready;        // 传感器就绪状态
        bool data_ready;          // 数据就绪状态
        bool fifo_overflow;       // FIFO溢出状态
        bool fifo_watermark;      // FIFO水印状态
        uint16_t fifo_count;      // FIFO当前数据计数
        uint8_t interrupt_status; // 中断状态寄存器值
        uint32_t error_count;     // 错误计数
    } icm42688_device_status_t;

    /**
     * @brief 设备校准数据
     */
    typedef struct
    {
        float accel_offset[3]; // 加速度计偏移量
        float gyro_offset[3];  // 陀螺仪偏移量
        float accel_scale[3];  // 加速度计缩放因子
        float gyro_scale[3];   // 陀螺仪缩放因子
        bool calibrated;       // 校准标志
    } icm42688_calibration_data_t;
    /* ========================================================================== */
    /*                          设备句柄和状态结构体                                */
    /* ========================================================================== */

    /**
     * @brief 设备句柄
     */
    typedef struct
    {
        /* 通信接口 */
        icm42688_comm_config_t comm; // 通信配置
        uint8_t current_bank;        // 当前寄存器Bank

        /* 配置参数 */
        icm42688_sensor_config_t config;   // 传感器配置
        icm42688_calibration_data_t calib; // 校准数据

        /* 状态信息 */
        icm42688_device_status_t status; // 设备状态
        bool initialized;                // 初始化标志
        uint8_t whoami;                  // 设备ID

        /* 回调函数 */
        icm42688_int_callback_fn int1_callback; // INT1中断回调
        icm42688_int_callback_fn int2_callback; // INT2中断回调
        void *callback_user_data;               // 回调用户数据

        /* 内部数据 */
        uint32_t last_read_time; // 最后读取时间
        uint32_t sample_count;   // 采样计数
    } icm42688_device_t;

    /* ========================================================================== */
    /*                          设备初始化和管理函数                             */
    /* ========================================================================== */

    /**
     * @brief 初始化ICM-42688设备
     * @param dev 设备句柄指针
     * @param comm_config 通信配置
     * @return 错误码
     */
    icm42688_err_t icm42688_init(icm42688_device_t *dev,
                                 const icm42688_comm_config_t *comm_config);

    /**
     * @brief 反初始化设备，释放资源
     * @param dev 设备句柄指针
     * @return 错误码
     */
    icm42688_err_t icm42688_deinit(icm42688_device_t *dev);

    /**
     * @brief 复位设备
     * @param dev 设备句柄指针
     * @return 错误码
     */
    icm42688_err_t icm42688_reset(icm42688_device_t *dev);

    /**
     * @brief 检查设备连接状态
     * @param dev 设备句柄指针
     * @return true-设备正常，false-设备异常
     */
    bool icm42688_check_connection(icm42688_device_t *dev);

    /* ========================================================================== */
    /*                          传感器配置函数                                   */
    /* ========================================================================== */

    /**
     * @brief 配置传感器参数
     * @param dev 设备句柄指针
     * @param config 传感器配置
     * @return 错误码
     */
    icm42688_err_t icm42688_config_sensor(icm42688_device_t *dev,
                                          const icm42688_sensor_config_t *config);

    /**
     * @brief 设置传感器工作模式
     * @param dev 设备句柄指针
     * @param gyro_mode 陀螺仪模式
     * @param accel_mode 加速度计模式
     * @return 错误码
     */
    icm42688_err_t icm42688_set_sensor_mode(icm42688_device_t *dev,
                                            icm42688_sensor_mode_t gyro_mode,
                                            icm42688_sensor_mode_t accel_mode);

    /**
     * @brief 设置陀螺仪参数
     * @param dev 设备句柄指针
     * @param fs 满量程范围
     * @param odr 输出数据速率
     * @return 错误码
     */
    icm42688_err_t icm42688_set_gyro_config(icm42688_device_t *dev,
                                            icm42688_gyro_fs_t fs,
                                            icm42688_odr_t odr);

    /**
     * @brief 设置加速度计参数
     * @param dev 设备句柄指针
     * @param fs 满量程范围
     * @param odr 输出数据速率
     * @return 错误码
     */
    icm42688_err_t icm42688_set_accel_config(icm42688_device_t *dev,
                                             icm42688_accel_fs_t fs,
                                             icm42688_odr_t odr);

    /* ========================================================================== */
    /*                          数据读取函数                                     */
    /* ========================================================================== */

    /**
     * @brief 读取传感器原始数据
     * @param dev 设备句柄指针
     * @param raw_data 原始数据输出
     * @return 错误码
     */
    icm42688_err_t icm42688_read_raw_data(icm42688_device_t *dev,
                                          icm42688_raw_data_t *raw_data);

    /**
     * @brief 读取并转换传感器数据
     * @param dev 设备句柄指针
     * @param sensor_data 传感器数据输出
     * @return 错误码
     */
    icm42688_err_t icm42688_read_sensor_data(icm42688_device_t *dev,
                                             icm42688_sensor_data_t *sensor_data);

    /**
     * @brief 检查数据是否就绪
     * @param dev 设备句柄指针
     * @param data_ready 数据就绪标志输出
     * @return 错误码
     */
    icm42688_err_t icm42688_check_data_ready(icm42688_device_t *dev, bool *data_ready);

    /**
     * @brief 等待数据就绪（阻塞式）
     * @param dev 设备句柄指针
     * @param timeout_ms 超时时间（毫秒）
     * @return 错误码
     */
    icm42688_err_t icm42688_wait_data_ready(icm42688_device_t *dev, uint32_t timeout_ms);

    /* ========================================================================== */
    /*                          FIFO相关函数                                     */
    /* ========================================================================== */

    /**
     * @brief 配置FIFO
     * @param dev 设备句柄指针
     * @param config FIFO配置
     * @return 错误码
     */
    icm42688_err_t icm42688_config_fifo(icm42688_device_t *dev,
                                        const icm42688_sensor_config_t *config);

    /**
     * @brief 读取FIFO数据包数量
     * @param dev 设备句柄指针
     * @param packet_count 数据包数量输出
     * @return 错误码
     */
    icm42688_err_t icm42688_get_fifo_packet_count(icm42688_device_t *dev,
                                                  uint16_t *packet_count);

    /**
     * @brief 读取FIFO数据
     * @param dev 设备句柄指针
     * @param packets 数据包缓冲区
     * @param max_packets 最大数据包数
     * @param read_packets 实际读取的数据包数
     * @return 错误码
     */
    icm42688_err_t icm42688_read_fifo(icm42688_device_t *dev,
                                      icm42688_fifo_packet_t *packets,
                                      uint16_t max_packets,
                                      uint16_t *read_packets);

    /**
     * @brief 清空FIFO
     * @param dev 设备句柄指针
     * @return 错误码
     */
    icm42688_err_t icm42688_flush_fifo(icm42688_device_t *dev);

    /* ========================================================================== */
    /*                          中断配置函数                                     */
    /* ========================================================================== */

    /**
     * @brief 配置中断引脚
     * @param dev 设备句柄指针
     * @param int_num 中断号（1或2）
     * @param mode 中断模式
     * @param drive 驱动方式
     * @param polarity 极性
     * @return 错误码
     */
    icm42688_err_t icm42688_config_interrupt_pin(icm42688_device_t *dev,
                                                 uint8_t int_num,
                                                 icm42688_int_mode_t mode,
                                                 icm42688_int_drive_t drive,
                                                 icm42688_int_polarity_t polarity);

    /**
     * @brief 使能中断源
     * @param dev 设备句柄指针
     * @param int_num 中断号（1或2）
     * @param int_source 中断源掩码
     * @return 错误码
     */
    icm42688_err_t icm42688_enable_interrupt_source(icm42688_device_t *dev,
                                                    uint8_t int_num,
                                                    uint8_t int_source);

    /**
     * @brief 设置中断回调函数
     * @param dev 设备句柄指针
     * @param int_num 中断号（1或2）
     * @param callback 回调函数
     * @param user_data 用户数据
     * @return 错误码
     */
    icm42688_err_t icm42688_set_interrupt_callback(icm42688_device_t *dev,
                                                   uint8_t int_num,
                                                   icm42688_int_callback_fn callback,
                                                   void *user_data);

    /**
     * @brief 读取中断状态
     * @param dev 设备句柄指针
     * @param status 中断状态输出
     * @return 错误码
     */
    icm42688_err_t icm42688_read_interrupt_status(icm42688_device_t *dev,
                                                  uint8_t *status);

    /* ========================================================================== */
    /*                          APEX功能函数                                     */
    /* ========================================================================== */

    /**
     * @brief 使能APEX功能
     * @param dev 设备句柄指针
     * @param feature APEX功能类型
     * @param enable 使能标志
     * @return 错误码
     */
    icm42688_err_t icm42688_enable_apex_feature(icm42688_device_t *dev,
                                                icm42688_apex_feature_t feature,
                                                bool enable);

    /**
     * @brief 配置计步器
     * @param dev 设备句柄指针
     * @param sensitivity 灵敏度设置
     * @return 错误码
     */
    icm42688_err_t icm42688_config_pedometer(icm42688_device_t *dev,
                                             uint8_t sensitivity);

    /**
     * @brief 读取计步器数据
     * @param dev 设备句柄指针
     * @param pedometer_data 计步器数据输出
     * @return 错误码
     */
    icm42688_err_t icm42688_read_pedometer_data(icm42688_device_t *dev,
                                                icm42688_pedometer_data_t *pedometer_data);

    /**
     * @brief 配置敲击检测
     * @param dev 设备句柄指针
     * @param sensitivity 灵敏度设置
     * @return 错误码
     */
    icm42688_err_t icm42688_config_tap_detection(icm42688_device_t *dev,
                                                 uint8_t sensitivity);

    /**
     * @brief 读取敲击检测数据
     * @param dev 设备句柄指针
     * @param tap_data 敲击数据输出
     * @return 错误码
     */
    icm42688_err_t icm42688_read_tap_data(icm42688_device_t *dev,
                                          icm42688_tap_data_t *tap_data);

    /* ========================================================================== */
    /*                          校准和自检函数                                   */
    /* ========================================================================== */

    /**
     * @brief 执行传感器自检
     * @param dev 设备句柄指针
     * @param gyro_self_test 陀螺仪自检结果
     * @param accel_self_test 加速度计自检结果
     * @return 错误码
     */
    icm42688_err_t icm42688_self_test(icm42688_device_t *dev,
                                      bool *gyro_self_test,
                                      bool *accel_self_test);

    /**
     * @brief 校准传感器（需要设备静止）
     * @param dev 设备句柄指针
     * @param samples 采样数量
     * @param timeout_ms 超时时间
     * @return 错误码
     */
    icm42688_err_t icm42688_calibrate_sensor(icm42688_device_t *dev,
                                             uint32_t samples,
                                             uint32_t timeout_ms);

    /**
     * @brief 设置用户偏移量
     * @param dev 设备句柄指针
     * @param axis 轴选择（0:X, 1:Y, 2:Z）
     * @param gyro_offset 陀螺仪偏移量
     * @param accel_offset 加速度计偏移量
     * @return 错误码
     */
    icm42688_err_t icm42688_set_user_offset(icm42688_device_t *dev,
                                            uint8_t axis,
                                            int16_t gyro_offset,
                                            int16_t accel_offset);

    /* ========================================================================== */
    /*                          工具和状态函数                                   */
    /* ========================================================================== */

    /**
     * @brief 读取设备状态
     * @param dev 设备句柄指针
     * @param status 设备状态输出
     * @return 错误码
     */
    icm42688_err_t icm42688_get_device_status(icm42688_device_t *dev,
                                              icm42688_device_status_t *status);

    /**
     * @brief 读取芯片温度
     * @param dev 设备句柄指针
     * @param temperature 温度输出 [°C]
     * @return 错误码
     */
    icm42688_err_t icm42688_read_temperature(icm42688_device_t *dev,
                                             float *temperature);

    /**
     * @brief 获取设备信息
     * @param dev 设备句柄指针
     * @param info 设备信息字符串缓冲区
     * @param len 缓冲区长度
     * @return 错误码
     */
    icm42688_err_t icm42688_get_device_info(icm42688_device_t *dev,
                                            char *info, uint32_t len);

    /**
     * @brief 转换原始加速度数据到物理量
     * @param raw_data 原始数据
     * @param fs 满量程范围
     * @param accel 加速度输出 [m/s²]
     */
    void icm42688_convert_accel_data(int16_t raw_data, icm42688_accel_fs_t fs, float *accel);

    /**
     * @brief 转换原始陀螺仪数据到物理量
     * @param raw_data 原始数据
     * @param fs 满量程范围
     * @param gyro 角速度输出 [rad/s]
     */
    void icm42688_convert_gyro_data(int16_t raw_data, icm42688_gyro_fs_t fs, float *gyro);

    /**
     * @brief 获取错误码描述字符串
     * @param err 错误码
     * @return 错误描述字符串
     */
    const char *icm42688_get_error_string(icm42688_err_t err);

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_REGS_H */