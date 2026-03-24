/**
 ******************************************************************************
 * @file    SC7A20_reg.h
 * @brief   SC7A20HTR 加速度计寄存器定义头文件
 ******************************************************************************
 * @note    本文件内定义了 SC7A20HTR 加速度计的寄存器地址、
 *          寄存器位域结构体、寄存器位域结构体定义等。
 ******************************************************************************
 */

#ifndef SC7A20_REG_H
#define SC7A20_REG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========================== 设备基本信息 ========================== */
#define SC7A20_CHIP_ID     0x11 // 设备ID寄存器预期值
#define SC7A20_VERSION_VAL 0x28 // 版本号寄存器预期值
// I2C地址
#define SC7A20_I2C_ADDR_L 0x18 //  SDO接逻辑低
#define SC7A20_I2C_ADDR_H 0x19 //  SDO悬空/接逻辑高

/* ========================== 核心寄存器地址 ========================== */
/* 设备识别 */
#define SC7A20_WHO_AM_I 0x0F // 设备ID寄存器
#define SC7A20_VERSION  0x70 // 版本号寄存器
/* 控制寄存器组 */
#define SC7A20_CTRL0      0x1F // 控制寄存器0（模式控制）
#define SC7A20_CTRL1      0x20 // 控制寄存器1（加速度计配置）
#define SC7A20_CTRL2      0x21 // 控制寄存器2（滤波器配置）
#define SC7A20_CTRL3      0x22 // 控制寄存器3（中断配置）
#define SC7A20_CTRL4      0x23 // 控制寄存器4（数据设置）
#define SC7A20_CTRL5      0x24 // 控制寄存器5
#define SC7A20_CTRL6      0x25 // 控制寄存器6

#define SC7A20_SOFT_RESET 0x68 // 软复位 (寄存器写入0xA5，复位整个电路，数据清零)
#define SC7A20_I2C_CTRL   0x6F // I2C控制寄存器

#define SC7A20_SPI_CTRL   0x0E // SPI控制寄存器
/* 状态寄存器 */
#define SC7A20_DRDY_STATUS 0x27 // 状态寄存器
/* ========================== 数据输出寄存器 ========================== */
/* 加速度计输出 */
#define SC7A20_OUTX_L 0x28 // 加速度计X轴低字节
#define SC7A20_OUTX_H 0x29 // 加速度计X轴高字节
#define SC7A20_OUTY_L 0x2A // 加速度计Y轴低字节
#define SC7A20_OUTY_H 0x2B // 加速度计Y轴高字节
#define SC7A20_OUTZ_L 0x2C // 加速度计Z轴低字节
#define SC7A20_OUTZ_H 0x2D // 加速度计Z轴高字节

/* 实时的X轴加速度计值 */
#define SC7A20_OUTX_New_L 0x61 // 实时X轴加速度计低字节
#define SC7A20_OUTX_New_H 0x62 // 实时X轴加速度计高字节
/* 实时Y轴加速度计值 */
#define SC7A20_OUTY_New_L 0x63 // 实时Y轴加速度计低字节
#define SC7A20_OUTY_New_H 0x64 // 实时Y轴加速度计高字节
/* 实时Z轴加速度计值 */
#define SC7A20_OUTZ_New_L 0x65 // 实时Z轴加速度计低字节
#define SC7A20_OUTZ_New_H 0x66 // 实时Z轴加速度计高字节
/* ========================== FIFO控制寄存器 ========================== */
#define SC7A20_FIFO_CTRL 0x2E // FIFO控制寄存器
#define SC7A20_FIFO_SRC  0x2F // FIFO状态寄存器
#define SC7A20_FIFO_DATA 0x69 // FIFO数据寄存器
/*
 * 读取 SC7A20_FIFO_DATA 寄存器相当于是读取FIFO数据，读取数据顺序是X轴、Y轴、Z轴；
 * 可以根据0x2F寄存器值计算FIFO组数，然后组数*3作为读取0x69的次数；
 * FIFO_MODE=0相当于顺序是28h,29h,2Ah,2Bh,2Ch,2Dh；
 * FIFO_MODE=1相当于顺序是28h,2Ah,2Ch。
 */

/* ========================== 中断控制寄存器 ========================== */
#define SC7A20_INT1_CFG 0x30 // INT1配置寄存器
#define SC7A20_INT1_SRC 0x31 // INT1状态寄存器
#define SC7A20_INT1_THS 0x32 // INT1阈值寄存器
#define SC7A20_INT1_DUR 0x33 // INT1持续时间寄存器

#define SC7A20_INT2_CFG 0x34 // INT2配置寄存器
#define SC7A20_INT2_SRC 0x35 // INT2状态寄存器
#define SC7A20_INT2_THS 0x36 // INT2阈值寄存器
#define SC7A20_INT2_DUR 0x37 // INT2持续时间寄存器
/* ========================== 运动检测寄存器 ========================== */
#define SC7A20_CLICK_CRTL   0x38 // 敲击控制寄存器
#define SC7A20_CLICK_SRC    0x39 // 敲击状态寄存器
#define SC7A20_CLICK_COEFF1 0x3A // 敲击系数寄存器1
#define SC7A20_CLICK_COEFF2 0x3B // 敲击系数寄存器2
#define SC7A20_CLICK_COEFF3 0x3C // 敲击系数寄存器3
#define SC7A20_CLICK_COEFF4 0x3D // 敲击系数寄存器4

#define SC7A20_DIG_CTRL     0x57 // 数字功能控制寄存器

/* ========================== 寄存器位域结构体 ========================== */

/* CTRL0 (0x1F): 模式控制 */
typedef union {
    struct {
        uint8_t HR : 1;   // 工作模式控制位 (0:低功耗, 1:高分辨率  -->请参考20h寄存器说明)
        uint8_t DLPF : 1; // 数字低通滤波器控制位高位。
        uint8_t : 1;      // 保留位
        uint8_t : 1;      // 保留位
        uint8_t OSR : 3;  // 数据更新速率控制位 (000: ODR, 001: ODR/2, 010: ODR/4, 011:ODR/8, 100:ODR/16, 101~111: ODR/32)
        uint8_t : 1;      // 保留位
    } bit;
    uint8_t reg;
} sc7a20_ctrl0_t;

/* CTRL1 (0x20): 加速度计配置 */
/*
 * 数据输出率的配置
 * 0000: 电源关断模式           0001: 全工作模式(1.56Hz)
 * 0010: 全工作模式(12.5Hz)     0011: 全工作模式(25Hz)
 * 0100: 全工作模式(50Hz)       0101: 全工作模式(100Hz)
 * 0110: 全工作模式(200Hz)      0111: 全工作模式(400Hz)
 * 1000: 全工作模式(800Hz)      1001: 高性能模式(1.48kHz)
 * 1010: 高性能模式(2.66kHz)    1011: 高性能模式(4.434kHz)
 *
 * 工作模式配置
 * HR=0, LPen=0: 正常模式
 * HR=0, LPen=1: 低功耗模式
 * HR=1, LPen=0: 高性能模式
 * HR=1, LPen=1: 增强模式
 *
 */

typedef union {
    struct {
        uint8_t Xen : 1;  // X轴使能位
        uint8_t Yen : 1;  // Y轴使能位
        uint8_t Zen : 1;  // Z轴使能位
        uint8_t LPen : 1; // 低功耗模式使能位
        uint8_t ODR : 4;  // 数据率选择
    } bit;
    uint8_t reg;
} sc7a20_ctrl1_t;

/* CTRL2 (0x21): 滤波器配置 */
/*
 * 数据输出率的配置
 * HPCF | Ft[Hz]@12.5Hz | Ft[Hz]@25Hz | Ft[Hz]@50Hz | Ft[Hz]@100Hz | Ft[Hz]@200Hz | Ft[Hz]@400Hz
 *  00      0.8              2               4               8           16              32
 *  01      0.32            0.8              2               4            8              16
 *  10      0.04            0.1             0.2             0.5           1               2
 *  11      0.02            0.05            0.1             0.2          0.5              1
 */
typedef union {
    struct {
        uint8_t HPIS1 : 1;    // 中断AOI1功能高通滤波使能
        uint8_t HPIS2 : 1;    // 中断AOI2功能高通滤波使能
        uint8_t HP_reset : 1; // 高通滤波器复位
        uint8_t FDS : 1;      // 数据滤波选择
        uint8_t HPCF : 2;     // 高通截止频率选择
        uint8_t HDS : 1;      // 高通滤波器数据选择
        uint8_t : 1;          // 保留位
    } bit;
    uint8_t reg;
} sc7a20_ctrl2_t;

/* CTRL3 (0x22): 中断配置 */
typedef union {
    struct {
        uint8_t fifo_mode : 1;    // FIFO模式选择(0：12位数据模式；1：8位数据模式)
        uint8_t int1_overrun : 1; // FIFO 溢出中断在INT1上
        uint8_t int1_wtm : 1;     // FIFO_WTM中断在INT1上
        uint8_t int1_drdy : 1;    // DRDY中断在INT1上
        uint8_t int1_aoi2 : 1;    // AOI2中断在INT1上
        uint8_t int1_aoi1 : 1;    // AOI1中断在INT1上
        uint8_t int1_click : 1;   // CLICK中断在INT1上
    } bit;
    uint8_t reg;
} sc7a20_ctrl3_t;

/* CTRL4 (0x23):  数据设置 */
typedef union {
    struct {
        uint8_t sim : 1;  // SPI 串行接口模式配置
        uint8_t st : 2;   // 自测试使能
        uint8_t DLPF : 1; // 数字低通滤波器控制位低位
        uint8_t fs : 2;   // 全量程选择
        uint8_t BLE : 1;  // 数据字节序选择 (0：低字节数据在低地址；1：高字节数据在低地址)
        uint8_t BDU : 1;  // 块数据更新 (0：连续更新；1：输出数据寄存器不更新直到MSB和LSB被读取)
    } bit;
    uint8_t reg;
} sc7a20_ctrl4_t;

/* CTRL5 (0x24): 配置寄存器 5 */
typedef union {
    struct {
        uint8_t D4D_INT2 : 1; // 4D使能：在INT2管脚上使能4D检测，同时要把中断2配置寄存器中的6D为置1。
        uint8_t LIR_INT2 : 1; // 锁存中断2配置寄存器上指定的中断响应 (0：不锁存中断信号；1：锁存中断信号)
        uint8_t D4D_INT1 : 1; // 4D使能：在INT1管脚上使能4D检测，同时要把中断1配置寄存器中的6D为置1。
        uint8_t LIR_INT1 : 1; // 锁存中断1配置寄存器上指定的中断响应 (0：不锁存中断信号；1：锁存中断信号)
        uint8_t AOI_EN : 1;   // AOI中断禁止位
        uint8_t FIFO_EN : 2;  // FIFO使能
        uint8_t boot : 1;     // 重载修调值
    } bit;
    uint8_t reg;
} sc7a20_ctrl5_t;

/* CTRL6 (0x25): 中断控制 */
typedef union {
    struct {
        uint8_t INT_PP_OD : 1; // INT1和INT2推挽输出或开漏输出选择位
        uint8_t H_LACTIVE : 1; // 中断引脚默认电平控制位
        uint8_t CS_PU_EN : 1;  // CS引脚上拉电阻使能位
        uint8_t I2_DRDY : 1;   // DRDY中断在INT2上
        uint8_t I2_BOOT : 1;   // BOOT状态在INT2上
        uint8_t I2_AOI2 : 1;   // AOI2中断在INT2上
        uint8_t I2_AOI1 : 1;   // AOI1中断在INT2上
        uint8_t I2_CLICK : 1;  // CLICK中断在INT2上
    } bit;
    uint8_t reg;
} sc7a20_ctrl6_t;

/* DRDY_STATUS (0x27): 状态寄存器 */
typedef union {
    struct {
        uint8_t XDA : 1;   // X轴数据可用
        uint8_t YDA : 1;   // Y轴数据可用
        uint8_t ZDA : 1;   // Z轴数据可用
        uint8_t ZYXDA : 1; // X，Y和Z三个轴新的数据全都转换完成
        uint8_t XOR : 1;   // X轴新的数据已经覆盖老的数据
        uint8_t YOR : 1;   // Y轴新的数据已经覆盖老的数据
        uint8_t ZOR : 1;   // Z轴新的数据已经覆盖老的数据
        uint8_t ZYXOR : 1; // X，Y和Z三个轴新的数据至少有一个已经覆盖老的数据。
    } bit;
    uint8_t reg;
} sc7a20_drdy_status_t;

/*  FIFO_CTRL (0x2E): FIFO控制寄存器 */
typedef union {
    struct {
        uint8_t FTH : 5; // FIFO功能WTM阈值设置
        uint8_t TR : 1;  // FIFO触发模式选择(0:AOI1中断作为FIFO触发模式中断事件输入, 1:AOI2中断作为FIFO触发模式中断事件输入 )
        uint8_t FM : 2;  // FIFO模式选择 (00:旁路模式, 01:FIFO模式, 10:流模式, 11:触发模式)
    } bit;
    uint8_t reg;
} sc7a20_fifo_ctrl_t;

/*  FIFO_SRC (0x2F):  FIFO状态寄存器 */
typedef union {
    struct {
        uint8_t FSS : 5;   // 在FIFO中未读取数据的组数
        uint8_t EMPTY : 1; // 当FIFO中的数据全部被读取或者FIFO数据个数为0时，EMPTY位置“1”
        uint8_t OVER : 1;  // FIFO溢出标志 (1:发生溢出)
        uint8_t WTM : 1;   // 当FIFO中的数据个数超过设定阈值时，WTM位置“1”
    } bit;
    uint8_t reg;
} sc7a20_fifo_src_t;

/*  INT1_CFG (0x30):  中断1配置寄存器 */
/*
 * AOI  | 6D  | 中断模式
 *  0      0    或中断事件
 *  0      1    6个方向运动识别
 *  1      0    与中断事件
 *  1      1    6个方向位置检测
 */

typedef union {
    struct {
        uint8_t XLIE_XDOWNE : 1; // X轴低事件中断或者X轴方向检测中断使能
        uint8_t XHIE_XUPE : 1;   // X轴高事件中断或者X轴方向检测中断使能
        uint8_t YLIE_YDOWNE : 1; // Y轴低事件中断或者Y轴方向检测中断使能
        uint8_t YHIE_XUPE : 1;   // Y轴高事件中断或者Y轴方向检测中断使能
        uint8_t ZLIE_ZDOWNE : 1; // Z轴低事件中断或者Z轴方向检测中断使能
        uint8_t ZHIE_ZUPE : 1;   // Z轴高事件中断或者Z轴方向检测中断使能
        uint8_t _6D : 1;         // 6D方向检测使
        uint8_t AOI : 1;         // 中断逻辑模式选择 (0:OR逻辑, 1:AND逻辑)
    } bit;
    uint8_t reg;
} sc7a20_int1_cfg_t;

/*  INT1_SRC (0x31):  中断1状态寄存器 */
typedef union {
    struct {
        uint8_t XL : 1; // X轴低 (0：没有中断，1：X轴低事件已经产生)
        uint8_t XH : 1; // X轴高 (0：没有中断，1：X轴高事件已经产生)
        uint8_t YL : 1; // Y轴低 (0：没有中断，1：Y轴低事件已经产生)
        uint8_t YH : 1; // Y轴高 (0：没有中断，1：Y轴高事件已经产生)
        uint8_t ZL : 1; // Z轴低 (0：没有中断，1：Z轴低事件已经产生)
        uint8_t ZH : 1; // Z轴高 (0：没有中断，1：Z轴高事件已经产生)
        uint8_t IA : 1; // 中断激活 (0：没有中断，1：中断已经产生)
        uint8_t : 1;    // 保留
    } bit;
    uint8_t reg;
} sc7a20_int1_src_t;

/*  INT2_THS (0x32):  中断1阈值寄存器 */
typedef union {
    struct {
        uint8_t THS : 7; // 中断1阈值
        uint8_t : 1;     // 保留
    } bit;
    uint8_t reg;
} sc7a20_int1_ths_t;

/*  INT1_DUR (0x33):  中断1持续时间 */
typedef union {
    struct {
        uint8_t D : 7; // 持续时间计数值
        uint8_t : 1;   // 保留
    } bit;
    uint8_t reg;
} sc7a20_int1_dur_t;

/*  INT2_CFG (0x34):  中断2配置寄存器 */
/*
 * AOI  | 6D  | 中断模式
 *  0      0    或中断事件
 *  0      1    6个方向运动识别
 *  1      0    与中断事件
 *  1      1    6个方向位置检测
 */

typedef union {
    struct {
        uint8_t XLIE_XDOWNE : 1; // X轴低事件中断或者X轴方向检测中断使能
        uint8_t XHIE_XUPE : 1;   // X轴高事件中断或者X轴方向检测中断使能
        uint8_t YLIE_YDOWNE : 1; // Y轴低事件中断或者Y轴方向检测中断使能
        uint8_t YHIE_XUPE : 1;   // Y轴高事件中断或者Y轴方向检测中断使能
        uint8_t ZLIE_ZDOWNE : 1; // Z轴低事件中断或者Z轴方向检测中断使能
        uint8_t ZHIE_ZUPE : 1;   // Z轴高事件中断或者Z轴方向检测中断使能
        uint8_t _6D : 1;         // 6D方向检测使
        uint8_t AOI : 1;         // 中断逻辑模式选择 (0:OR逻辑, 1:AND逻辑)
    } bit;
    uint8_t reg;
} sc7a20_int2_cfg_t;

/*  INT2_SRC (0x35):  中断2状态寄存器 */
typedef union {
    struct {
        uint8_t XL : 1; // X轴低 (0：没有中断，1：X轴低事件已经产生)
        uint8_t XH : 1; // X轴高 (0：没有中断，1：X轴高事件已经产生)
        uint8_t YL : 1; // Y轴低 (0：没有中断，1：Y轴低事件已经产生)
        uint8_t YH : 1; // Y轴高 (0：没有中断，1：Y轴高事件已经产生)
        uint8_t ZL : 1; // Z轴低 (0：没有中断，1：Z轴低事件已经产生)
        uint8_t ZH : 1; // Z轴高 (0：没有中断，1：Z轴高事件已经产生)
        uint8_t IA : 1; // 中断激活 (0：没有中断，1：中断已经产生)
        uint8_t : 1;    // 保留
    } bit;
    uint8_t reg;
} sc7a20_int2_src_t;

/*  INT2_THS (0x36):  中断2阈值寄存器 */
typedef union {
    struct {
        uint8_t THS : 7; // 中断1阈值
        uint8_t : 1;     // 保留
    } bit;
    uint8_t reg;
} sc7a20_int2_ths_t;

/*  INT2_DUR (0x37):  中断2持续时间 */
typedef union {
    struct {
        uint8_t D : 7; // 持续时间计数值
        uint8_t : 1;   // 保留
    } bit;
    uint8_t reg;
} sc7a20_int2_dur_t;

/*  CLICK_CRTL (0x38):  敲击控制寄存器 */
typedef union {
    struct {
        uint8_t CLICK_Z_EN : 1; // Z轴敲击功能使能位
        uint8_t CLICK_Y_EN : 1; // Y轴敲击功能使能位
        uint8_t CLICK_X_EN : 1; // X轴敲击功能使能位
        uint8_t LIR_CLICK : 1;  // 敲击中断锁存使能位
        uint8_t CLICK_SEL : 1;  // 0：敲击事件不为0时输出中断信号；1：输出中断必须满足设置的敲击阈值个数才能输出中断，否则无中断输出
        uint8_t : 3;            // 保留
    } bit;
    uint8_t reg;
} sc7a20_click_crtl_t;

/*  CLICK_SRC (0x39):  敲击控制寄存器 */
typedef union {
    struct {
        uint8_t CLICK_SRC : 4; // 敲击检测中断状态值 0000：无敲击事件触发；0001：单击事件触发；
                               // 0010：双击事件触发；0011：三击事件触发；…   1111：十五击事件触发；
        uint8_t CLICK_SEL : 1; // 0：敲击事件小于等于设置最大敲击次数时输出中断
                               // 1：输出中断必须满足设置的敲击阈值个数才能输出中断，否则无中断输出
        uint8_t : 3;           // 保留
    } bit;
    uint8_t reg;
} sc7a20_click_src_t;

/*  CLICK_COEFF1 (0x3A):  敲击系数寄存器1 */
typedef union {
    struct {
        uint8_t SCTH1 : 3;   // 敲击时识别有效时的数据阈值1设置
        uint8_t PRE_NTH : 3; // 敲击前数据稳定阈值设置
        uint8_t PRE_QT : 2;  // 敲击前数据稳定时长设置
    } bit;
    uint8_t reg;
} sc7a20_click_coeff1_t;

/*  CLICK_COEFF2 (0x3B):  敲击系数寄存器2 */
typedef union {
    struct {
        uint8_t SCTH1T : 3; // 敲击过程中数据大于SCTH1阈值的时间上限设置
        uint8_t SCTH2 : 3;  // 敲击时识别有效时的数据阈值2设置
        uint8_t QT_MT : 2;  // 敲击前必须保证数据平稳的最小时间设置，需要大于该阈值
    } bit;
    uint8_t reg;
} sc7a20_click_coeff12_t;

/*  CLICK_COEFF3 (0x3C):  敲击系数寄存器3 */
typedef union {
    struct {
        uint8_t SCST : 3;  // 敲击事件后所允许的最大恢复平静时长设置
        uint8_t SCNTT : 2; // 满足敲击事件前数据平静条件后，允许数据大于数据噪声阈值的最大时长
        uint8_t : 3;       // 保留
    } bit;
    uint8_t reg;
} sc7a20_click_coeff13_t;

/*  CLICK_COEFF4 (0x3D):  敲击系数寄存器4 */
typedef union {
    struct {
        uint8_t MCNTH : 4; // 多击检测最大检测次数设置
        uint8_t SCMT : 4;  // 单击检测的最大允许时长
    } bit;
    uint8_t reg;
} sc7a20_click_coeff14_t;

/*  DIG_CTRL (0x57):  数字功能控制寄存器 */
typedef union {
    struct {
        uint8_t : 2;        // 保留
        uint8_t I2C_PU : 1; // SDA和SCL内部上拉电阻控制位 (禁止上拉电阻后，该引脚为浮空输入模式，请保证引脚外围电平确定，否则I²C通讯会异常)
        uint8_t SDO_PU : 1; // SDO内部上拉电阻控制位 (禁止上拉电阻后，该引脚为开漏模式，请保证引脚外围有上拉电阻)
        uint8_t : 4;        // 保留
    } bit;
    uint8_t reg;
} sc7a20_dig_ctrl_t;

/*  I2C_CTRL (0x6F):  数字功能控制寄存器 */
typedef union {
    struct {
        uint8_t : 2;        // 保留
        uint8_t I2C_UN : 1; // 0：IIC通信使能；1：IIC通信关闭，配置 SC7A20_SOFT_RESET(68h)为66h后，再配置本寄存器才能生效；
        uint8_t : 5;        // 保留
    } bit;
    uint8_t reg;
} sc7a20_i2c_ctrl_t;

/* SPI_CTRL (0x0E): SPI控制寄存器 */
typedef union {
    struct {
        uint8_t : 4;             // 保留位
        uint8_t ADR_SPI_AD6 : 1; // 地址选择 0：SPI通信访问地址00H~3FH；1：SPI通信访问地址40H~7FH
        uint8_t : 3;             // 保留位
    } bit;
    uint8_t reg;
} sc7a20_spi_ctrl_t;
/* ========================== 枚举类型定义 ========================== */
/* 加速度计ODR枚举 */
typedef enum {
    SC7A20_ACCEL_ODR_POWER_DOWN = 0,  // 0000: 电源关断模式
    SC7A20_ACCEL_ODR_1_56HZ     = 1,  // 0001: 全工作模式(1.56Hz)
    SC7A20_ACCEL_ODR_12_5HZ     = 2,  // 0010: 全工作模式(12.5Hz)
    SC7A20_ACCEL_ODR_25HZ       = 3,  // 0011: 全工作模式(25Hz)
    SC7A20_ACCEL_ODR_50HZ       = 4,  // 0100: 全工作模式(50Hz)
    SC7A20_ACCEL_ODR_100HZ      = 5,  // 0101: 全工作模式(100Hz)
    SC7A20_ACCEL_ODR_200HZ      = 6,  // 0110: 全工作模式(200Hz)
    SC7A20_ACCEL_ODR_400HZ      = 7,  // 0111: 全工作模式(400Hz)
    SC7A20_ACCEL_ODR_800HZ      = 8,  // 1000: 全工作模式(800Hz)
    SC7A20_ACCEL_ODR_1_48KHZ    = 9,  // 1001: 高性能模式(1.48kHz)
    SC7A20_ACCEL_ODR_2_66KHZ    = 10, // 1010: 高性能模式(2.66kHz)
    SC7A20_ACCEL_ODR_4_434KHZ   = 11, // 1011: 高性能模式(4.434kHz)
} sc7a20_accel_odr_t;

/* 加速度计量程枚举 */
typedef enum {
    SC7A20_ACCEL_FS_2G  = 0, // ±2g
    SC7A20_ACCEL_FS_4G  = 1, // ±4g
    SC7A20_ACCEL_FS_8G  = 2, // ±8g
    SC7A20_ACCEL_FS_16G = 3  // ±16g
} sc7a20_accel_fs_t;

/* fifomode 枚举 */
typedef enum {
    SC7A20_FIFO_BYPASS_MODE  = 0, // 00：By-Pass模式（旁路模式，即不使用FIFO功能）
    SC7A20_FIFO_FIFO_MODE    = 1, // 01:FIFO模式（缓存满未及时读取，新数据丢弃）
    SC7A20_FIFO_STREAM_MODE  = 2, // 10:Stream模式（缓存满后，最早数据丢弃，添加新数据）
    SC7A20_FIFO_TRIGGER_MODE = 3  // 11:触发模式（AOI1或者AOI2中断事件有效，从stream模式进入FIFO模式）
} sc7a20_fifo_mode_t;

#ifdef __cplusplus
}
#endif

#endif /* SC7A20_REG_H */
