/**
 ******************************************************************************
 * @file    qmi8658a_reg.h
 * @brief   QMI8658A IMU传感器寄存器定义（优化版）
 ******************************************************************************
 * @note    本文件已按功能模块重新组织，便于阅读和开发
 ******************************************************************************
 */

#ifndef QMI8658A_REG_H
#define QMI8658A_REG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* ========================== 设备基本信息 ========================== */
#define QMI8658A_ID 0x05 // 设备标识符

/* ========================== 核心寄存器地址 ========================== */
/* 设备识别 */
#define QMI8658A_WHO_AM_I 0x00 // 设备ID寄存器

/* 控制寄存器组 */
#define QMI8658A_CTRL1 0x02 // 控制寄存器1（接口配置）
#define QMI8658A_CTRL2 0x03 // 控制寄存器2（加速度计配置）
#define QMI8658A_CTRL3 0x04 // 控制寄存器3（陀螺仪配置）
#define QMI8658A_CTRL5 0x06 // 控制寄存器5（滤波器配置）
#define QMI8658A_CTRL7 0x08 // 控制寄存器7（传感器使能）
#define QMI8658A_CTRL8 0x09 // 控制寄存器8（运动检测控制）
#define QMI8658A_CTRL9 0x0A // 控制寄存器9（命令寄存器）

/* 状态寄存器 */
#define QMI8658A_STATUSINT 0x2D // 状态中断寄存器
#define QMI8658A_STATUS0 0x2E   // 状态中断寄存器
/* ========================== 校准寄存器组 ========================== */
#define QMI8658A_CAL1_L 0x0B // 校准寄存器1低字节
#define QMI8658A_CAL1_H 0x0C // 校准寄存器1高字节
#define QMI8658A_CAL2_L 0x0D // 校准寄存器2低字节
#define QMI8658A_CAL2_H 0x0E // 校准寄存器2高字节
#define QMI8658A_CAL3_L 0x0F // 校准寄存器3低字节
#define QMI8658A_CAL3_H 0x10 // 校准寄存器3高字节
#define QMI8658A_CAL4_L 0x11 // 校准寄存器4低字节
#define QMI8658A_CAL4_H 0x12 // 校准寄存器4高字节

/* ========================== 运动检测寄存器 ========================== */
#define QMI8658A_TAP_STATUS 0x59    // 敲击检测状态寄存器
#define QMI8658A_STEP_CNT_LOW 0x5A  // 计步器计数低字节
#define QMI8658A_STEP_CNT_MIDL 0x5B // 计步器计数中低字节
#define QMI8658A_STEP_CNT_HIGH 0x5C // 计步器计数高字节

/* ========================== FIFO控制寄存器 ========================== */
#define QMI8658A_FIFO_WTM_TH 0x13   // FIFO水位阈值
#define QMI8658A_FIFO_CTRL 0x14     // FIFO控制寄存器
#define QMI8658A_FIFO_SMPL_CNT 0x15 // FIFO样本计数寄存器
#define QMI8658A_FIFO_STATUS 0x16   // FIFO状态寄存器
#define QMI8658A_FIFO_DATA 0x17     // FIFO数据寄存器

/* ========================== 系统控制寄存器 ========================== */
#define QMI8658A_RESET 0x60 // 复位寄存器

/* ========================== 数据输出寄存器 ========================== */
/* 温度输出 */
#define QMI8658A_OUT_TEMP_L 0x33 // 温度数据低字节
#define QMI8658A_OUT_TEMP_H 0x34 // 温度数据高字节

/* 陀螺仪输出 */
#define QMI8658A_OUTX_L_G 0x3B // 陀螺仪X轴低字节
#define QMI8658A_OUTX_H_G 0x3C // 陀螺仪X轴高字节
#define QMI8658A_OUTY_L_G 0x3D // 陀螺仪Y轴低字节
#define QMI8658A_OUTY_H_G 0x3E // 陀螺仪Y轴高字节
#define QMI8658A_OUTZ_L_G 0x3F // 陀螺仪Z轴低字节
#define QMI8658A_OUTZ_H_G 0x40 // 陀螺仪Z轴高字节

/* 加速度计输出 */
#define QMI8658A_OUTX_L_A 0x35 // 加速度计X轴低字节
#define QMI8658A_OUTX_H_A 0x36 // 加速度计X轴高字节
#define QMI8658A_OUTY_L_A 0x37 // 加速度计Y轴低字节
#define QMI8658A_OUTY_H_A 0x38 // 加速度计Y轴高字节
#define QMI8658A_OUTZ_L_A 0x39 // 加速度计Z轴低字节
#define QMI8658A_OUTZ_H_A 0x3A // 加速度计Z轴高字节

/* ========================== 时间戳寄存器 ========================== */
#define QMI8658A_TIMESTAMP_LOW 0x30  // 时间戳低字节
#define QMI8658A_TIMESTAMP_MID 0x31  // 时间戳中字节
#define QMI8658A_TIMESTAMP_HIGH 0x32 // 时间戳高字节

/* ========================== 命令定义 ========================== */
#define CTRL_CMD_REQ_FIFO 0x05 // 请求FIFO数据命令
#define CTRL_CMD_RST_FIFO 0x04 // 复位FIFO命令
#define CTRL_CMD_ACK 0x00      // 命令确认

    /* ========================== 寄存器位域结构体 ========================== */
    /* CTRL1 (0x02): 加速度计配置 */
    typedef union
    {
        struct
        {
            uint8_t SensorDisable : 1; // 启用内部高速振荡器
            uint8_t reserved : 1;      // 保留位
            uint8_t FIFO_INT_SEL : 1;  // FIFO中断映射(（0:映射到INT2, 1:映射到INT1)
            uint8_t INT1_EN : 1;       // INT2引脚输启用
            uint8_t INT2_EN : 1;       // INT2引脚输启用
            uint8_t BE : 1;            // 大小端选择（0:小端, 1:大端)
            uint8_t ADDR_AI : 1;       // 地址自增 （0:不自增, 1:自增)
            uint8_t SIM : 1;           // 启用三线/四线SPI接口 (0:四线, 1:三线)
        } bit;
        uint8_t reg;
    } qmi8658a_ctrl1_t;

    /* CTRL2 (0x03): 加速度计配置 */
    typedef union
    {
        struct
        {
            uint8_t aODR : 4; // 输出数据率 (0000:关闭, 0001:12.5Hz, ...)
            uint8_t aFS : 3;  // 量程选择 (000:±2g, 001:±4g, 010:±8g, 011:±16g)
            uint8_t aST : 1;  // 自检使能 (0:禁用, 1:启用)
        } bit;
        uint8_t reg;
    } qmi8658a_ctrl2_t;

    /* CTRL3 (0x04): 陀螺仪配置 */
    typedef union
    {
        struct
        {
            uint8_t gODR : 4; // 输出数据率 (0000:关闭, 0001:12.5Hz, ...)
            uint8_t gFS : 3;  // 量程选择 (000:±16dps, 001:±32dps, 010:±64dps, 011:1±28dps...)
            uint8_t gST : 1;  // 自检使能 (0:禁用, 1:启用)
        } bit;
        uint8_t reg;
    } qmi8658a_ctrl3_t;

    /* CTRL5 (0x06): 滤波器配置 */
    typedef union
    {
        struct
        {
            uint8_t aLPF_EN : 1;   // 加速度计低通滤波器使能 (0:禁用, 1:启用)
            uint8_t aLPF_MODE : 2; // 加速度计低通滤波器模式
            uint8_t reserved1 : 1; // 保留位
            uint8_t gLPF_EN : 1;   // 陀螺仪低通滤波器使能 (0:禁用, 1:启用)
            uint8_t gLPF_MODE : 2; // 陀螺仪低通滤波器模式
            uint8_t reserved2 : 1; // 保留位
        } bit;
        uint8_t reg;
    } qmi8658a_ctrl5_t;

    /* CTRL7 (0x08): 传感器使能控制 */
    typedef union
    {
        struct
        {
            uint8_t aEN : 1;       // 加速度计使能 (0:禁用, 1:启用)
            uint8_t gEN : 1;       // 陀螺仪使能 (0:禁用, 1:启用)
            uint8_t reserved2 : 2; // 保留位
            uint8_t gSN : 1;       // 陀螺仪睡眠模式 (0:正常, 1:睡眠)
            uint8_t DRDY_DIS : 1;  // 数据就绪中断禁用 (0:启用, 1:禁用)
            uint8_t reserved1 : 1; // 保留位
            uint8_t syncSmpl : 1;  // 同步采样模式 (0:禁用, 1:启用)
        } bit;
        uint8_t reg;
    } qmi8658a_ctrl7_t;

    /* CTRL8 (0x09): 运动检测控制 */
    typedef union
    {
        struct
        {
            uint8_t Tap_EN : 1;               // 敲击检测使能 (0:禁用, 1:启用)
            uint8_t AnyMotion_EN : 1;         // 任意运动检测使能 (0:禁用, 1:启用)
            uint8_t NoMotion_EN : 1;          // 无运动检测使能 (0:禁用, 1:启用)
            uint8_t SigMotion_EN : 1;         // 显著运动检测使能 (0:禁用, 1:启用)
            uint8_t Pedo_EN : 1;              // 计步器使能 (0:禁用, 1:启用)
            uint8_t reserved : 1;             // 保留位
            uint8_t ACTIVITY_INT_SEL : 1;     // 活动中断选择 (0:INT2, 1:INT1)
            uint8_t CTRL9_HandShake_Type : 1; // CTRL9握手类型 (0:状态位, 1:INT引脚)
        } bit;
        uint8_t reg;
    } qmi8658a_ctrl8_t;

    /* STATUSINT (0x2D): 状态中断寄存器 */
    typedef union
    {
        struct
        {
            uint8_t Avail : 1;    // 数据就绪标志 (1:数据可用)
            uint8_t Locked : 1;   // 数据锁定状态 (1:数据已锁定)
            uint8_t reserved : 5; // 保留位
            uint8_t CmdDone : 1;  // CTRL9命令完成标志 (1:命令执行完成)
        } bit;
        uint8_t reg;
    } qmi8658a_statusint_t;

    /* FIFO_CTRL (0x14): FIFO控制寄存器 */
    typedef union
    {
        struct
        {
            uint8_t FIFO_MODE : 2;    // FIFO工作模式 (00:旁路, 01:FIFO, 10:流模式)
            uint8_t FIFO_SIZE : 2;    // FIFO大小 (00:16样本, 01:32样本, 10:64样本, 11:128样本)
            uint8_t reserved : 3;     // 保留位
            uint8_t FIFO_RD_MODE : 1; // FIFO读模式 (0:禁用, 1:启用)
        } bit;
        uint8_t reg;
    } qmi8658a_fifo_ctrl_t;

    /* FIFO_STATUS (0x16): FIFO状态寄存器 */
    typedef union
    {
        struct
        {
            uint8_t FIFO_FULL : 1;         // FIFO满标志 (1:FIFO已满)
            uint8_t FIFO_WTM : 1;          // FIFO水位标记触发 (1:达到水位标记)
            uint8_t FIFO_OVFLOW : 1;       // FIFO溢出标志 (1:发生溢出)
            uint8_t FIFO_NOT_EMPTY : 1;    // FIFO非空标志 (1:FIFO非空)
            uint8_t reserved : 2;          // 保留位
            uint8_t FIFO_SMPL_CNT_MSB : 2; // FIFO样本计数高2位
        } bit;
        uint8_t reg;
    } qmi8658a_fifo_status_t;

    /* ========================== 枚举类型定义 ========================== */

    /* 加速度计量程枚举 */
    typedef enum
    {
        QMI8658A_ACCEL_FS_2G = 0, // ±2g
        QMI8658A_ACCEL_FS_4G = 1, // ±4g
        QMI8658A_ACCEL_FS_8G = 2, // ±8g
        QMI8658A_ACCEL_FS_16G = 3 // ±16g
    } qmi8658a_accel_fs_t;

    /* 陀螺仪量程枚举 */
    typedef enum
    {
        QMI8658A_GYRO_FS_16DPS = 0,   // ±16dps
        QMI8658A_GYRO_FS_32DPS = 1,   // ±32dps
        QMI8658A_GYRO_FS_64DPS = 2,   // ±64dps
        QMI8658A_GYRO_FS_128DPS = 3,  // ±128dps
        QMI8658A_GYRO_FS_256DPS = 4,  // ±256dps
        QMI8658A_GYRO_FS_512DPS = 5,  // ±512dps
        QMI8658A_GYRO_FS_1024DPS = 6, // ±1024dps
        QMI8658A_GYRO_FS_2048DPS = 7  // ±2048dps
    } qmi8658a_gyro_fs_t;

    /* 加速度计ODR枚举（根据手册修正） */
    typedef enum
    {
        /* Normal模式（0000-0111） */
        QMI8658A_ACCEL_ODR_7174_4HZ_NORMAL = 0, // 0000: 7174.4 Hz（Normal模式，100%占空比）
        QMI8658A_ACCEL_ODR_NA_0001 = 1,         // 0001: N/A（手册未定义有效ODR）
        QMI8658A_ACCEL_ODR_NA_0010 = 2,         // 0010: N/A（手册未定义有效ODR）
        QMI8658A_ACCEL_ODR_1000HZ_ACC_ONLY = 3, // 0011: 1000 Hz（仅加速度计模式）
        QMI8658A_ACCEL_ODR_500HZ_ACC_ONLY = 4,  // 0100: 500 Hz（仅加速度计模式）
        QMI8658A_ACCEL_ODR_250HZ_ACC_ONLY = 5,  // 0101: 250 Hz（仅加速度计模式）
        QMI8658A_ACCEL_ODR_125HZ_ACC_ONLY = 6,  // 0110: 125 Hz（仅加速度计模式）
        QMI8658A_ACCEL_ODR_62_5HZ_ACC_ONLY = 7, // 0111: 62.5 Hz（仅加速度计模式）

        /* Low Power模式（1000-1111） */
        QMI8658A_ACCEL_ODR_128HZ_LP = 8,    // 1000: 128 Hz（低功耗模式，100%占空比）
        QMI8658A_ACCEL_ODR_NA_1001 = 9,     // 1001: N/A（手册未定义有效ODR）
        QMI8658A_ACCEL_ODR_NA_1010 = 10,    // 1010: N/A（手册未定义有效ODR）
        QMI8658A_ACCEL_ODR_NA_1011 = 11,    // 1011: N/A（手册未定义有效ODR）
        QMI8658A_ACCEL_ODR_31_25HZ_LP = 12, // 1100: 31.25 Hz（低功耗模式，100%占空比）
        QMI8658A_ACCEL_ODR_21HZ_LP = 13,    // 1101: 21 Hz（低功耗模式，58%占空比）
        QMI8658A_ACCEL_ODR_11HZ_LP = 14,    // 1110: 11 Hz（低功耗模式，31%占空比）
        QMI8658A_ACCEL_ODR_3HZ_LP = 15      // 1111: 3 Hz（低功耗模式，8.5%占空比）
    } qmi8658a_accel_odr_t;

    /* 陀螺仪ODR枚举 */
    typedef enum
    {
        QMI8658A_GYRO_ODR_7174_4HZ = 0, // 0000: 7174.4 Hz（Normal模式，100%占空比）
        QMI8658A_GYRO_ODR_3587_2HZ = 1, // 0001: 3587.2 Hz
        QMI8658A_GYRO_ODR_1793_6HZ = 2, // 0010: 1793.6 Hz
        QMI8658A_GYRO_ODR_896_8HZ = 3,  // 0011: 896.8 Hz
        QMI8658A_GYRO_ODR_448_4HZ = 4,  // 0100: 448.4 Hz
        QMI8658A_GYRO_ODR_224_2HZ = 5,  // 0101: 224.2 Hz
        QMI8658A_GYRO_ODR_112_1HZ = 6,  // 0110: 112.1 Hz
        QMI8658A_GYRO_ODR_56_05HZ = 7   // 0111: 56.05 Hz
    } qmi8658a_gyro_odr_t;

    /* FIFO工作模式枚举 */
    typedef enum
    {
        QMI8658A_FIFO_MODE_BYPASS = 0,  // 旁路模式
        QMI8658A_FIFO_MODE_FIFO = 1,    // FIFO模式
        QMI8658A_FIFO_MODE_STREAM = 2,  // 流模式
        QMI8658A_FIFO_MODE_RESERVED = 3 // 保留
    } qmi8658a_fifo_mode_t;

    /* FIFO大小枚举 */
    typedef enum
    {
        QMI8658A_FIFO_SIZE_16SAMPLES = 0, // 16样本
        QMI8658A_FIFO_SIZE_32SAMPLES = 1, // 32样本
        QMI8658A_FIFO_SIZE_64SAMPLES = 2, // 64样本
        QMI8658A_FIFO_SIZE_128SAMPLES = 3 // 128样本
    } qmi8658a_fifo_size_t;

    /* ========================== 设备上下文结构 ========================== */

    typedef int32_t (*qmi8658a_write_ptr)(void *, uint8_t, uint8_t *, uint16_t);
    typedef int32_t (*qmi8658a_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);

    typedef struct
    {
        qmi8658a_write_ptr write_reg; // 寄存器写入函数指针
        qmi8658a_read_ptr read_reg;   // 寄存器读取函数指针
        void *handle;                 // 硬件句柄

        // 设备状态记录
        qmi8658a_accel_fs_t accel_fs; // 当前加速度计量程
        qmi8658a_gyro_fs_t gyro_fs;   // 当前陀螺仪量程
        uint8_t is_accel_enabled;     // 加速度计使能状态
        uint8_t is_gyro_enabled;      // 陀螺仪使能状态
    } qmi8658a_ctx_t;

    /* ========================== 函数声明 ========================== */

    // I2C通信接口
    int32_t qmi8658a_i2c_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
    int32_t qmi8658a_i2c_write(void *handle, uint8_t reg, uint8_t *data, uint16_t len);

    // 基础寄存器操作
    int32_t qmi8658a_read_reg(qmi8658a_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len);
    int32_t qmi8658a_write_reg(qmi8658a_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len);

    // 设备信息获取
    int32_t qmi8658a_device_id_get(qmi8658a_ctx_t *ctx, uint8_t *buff);

    // 系统控制
    int32_t qmi8658a_soft_reset(qmi8658a_ctx_t *ctx); // 软复位

    // 加速度计配置
    int32_t qmi8658a_accel_data_rate_set(qmi8658a_ctx_t *ctx, qmi8658a_accel_odr_t val);
    int32_t qmi8658a_accel_data_rate_get(qmi8658a_ctx_t *ctx, qmi8658a_accel_odr_t *val);
    int32_t qmi8658a_accel_full_scale_set(qmi8658a_ctx_t *ctx, qmi8658a_accel_fs_t val);
    int32_t qmi8658a_accel_full_scale_get(qmi8658a_ctx_t *ctx, qmi8658a_accel_fs_t *val);
    int32_t qmi8658a_accel_enable(qmi8658a_ctx_t *ctx, uint8_t enable);

    // 陀螺仪配置
    int32_t qmi8658a_gyro_data_rate_set(qmi8658a_ctx_t *ctx, qmi8658a_gyro_odr_t val);
    int32_t qmi8658a_gyro_data_rate_get(qmi8658a_ctx_t *ctx, qmi8658a_gyro_odr_t *val);
    int32_t qmi8658a_gyro_full_scale_set(qmi8658a_ctx_t *ctx, qmi8658a_gyro_fs_t val);
    int32_t qmi8658a_gyro_full_scale_get(qmi8658a_ctx_t *ctx, qmi8658a_gyro_fs_t *val);
    int32_t qmi8658a_gyro_enable(qmi8658a_ctx_t *ctx, uint8_t enable);

    // 滤波器配置
    int32_t qmi8658a_filter_config(qmi8658a_ctx_t *ctx, uint8_t accel_filter, uint8_t gyro_filter);

    // FIFO配置
    int32_t qmi8658a_fifo_config(qmi8658a_ctx_t *ctx, qmi8658a_fifo_mode_t mode, qmi8658a_fifo_size_t size);
    int32_t qmi8658a_fifo_read_mode_disable(qmi8658a_ctx_t *ctx);

    // 数据读取
    int32_t qmi8658a_temperature_get(qmi8658a_ctx_t *ctx, float *temp);
    int32_t qmi8658a_angular_rate_raw(qmi8658a_ctx_t *ctx, uint8_t *buff);
    int32_t qmi8658a_acceleration_raw(qmi8658a_ctx_t *ctx, uint8_t *buff);
    int32_t qmi8658a_timestamp_get(qmi8658a_ctx_t *ctx, uint32_t *timestamp);
    // 命令接口
    int32_t qmi8658a_ctrl9_command(qmi8658a_ctx_t *ctx, uint8_t cmd);

#ifdef __cplusplus
}
#endif

#endif /* QMI8658A_REG_H */
