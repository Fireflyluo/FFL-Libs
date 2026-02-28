# ICM42688P 6轴IMU驱动说明文档

## 1. 概述

ICM42688P是一款高性能6轴惯性测量单元（IMU），集成了3轴加速度计和3轴陀螺仪。支持多种量程和输出数据率配置，适用于运动检测、姿态估计、手势识别等应用场景。

### 1.1 主要特性

- **加速度计量程**：±2g, ±4g, ±8g, ±16g
- **陀螺仪量程**：±250dps, ±500dps, ±1000dps, ±2000dps
- **高数据率**：加速度计最高1.6kHz，陀螺仪最高8kHz
- **灵活的通信接口**：I2C/SPI（本驱动支持两种接口）
- **低功耗模式**：多种省电模式，适合电池供电应用
- **内置FIFO**：32字节FIFO缓冲区，减少主机处理器负载
- **中断功能**：支持数据就绪、运动检测等中断

### 1.2 驱动架构

```
ICM42688P Driver
├── Core Layer (icm42688p.c/h)    # 核心驱动实现
├── Register Layer (icm42688_reg.h) # 寄存器定义
└── Examples                        # 使用示例（参考demo目录）
```

## 2. 配置选项

### 2.1 设备配置结构

```c
typedef struct {
    uint8_t i2c_addr;              // I2C地址 (0x68 或 0x69)
    bool use_spi;                  // 是否使用SPI接口
    icm42688_accel_fs_t accel_fs;  // 加速度计量程
    icm42688_gyro_fs_t gyro_fs;    // 陀螺仪量程
    icm42688_odr_t accel_odr;      // 加速度计输出数据率
    icm42688_odr_t gyro_odr;       // 陀螺仪输出数据率
    bool enable_accel;             // 使能加速度计
    bool enable_gyro;              // 使能陀螺仪
} icm42688_config_t;
```

### 2.2 基本使用流程

```c
#include "icm42688p.h"

// 1. 初始化配置
icm42688_config_t config = {
    .i2c_addr = 0x68,
    .use_spi = false,
    .accel_fs = ICM42688_ACCEL_FS_4G,
    .gyro_fs = ICM42688_GYRO_FS_1000DPS,
    .accel_odr = ICM42688_ODR_100HZ,
    .gyro_odr = ICM42688_ODR_100HZ,
    .enable_accel = true,
    .enable_gyro = true
};

// 2. 初始化设备
if (icm42688_init(&config) != ICM42688_OK) {
    // 初始化失败处理
}

// 3. 读取传感器数据
icm42688_data_t sensor_data;
if (icm42688_read_data(&sensor_data) == ICM42688_OK) {
    // 处理加速度和角速度数据
    float accel_x = sensor_data.accel_x;
    float gyro_z = sensor_data.gyro_z;
}
```

## 3. 平台适配

### 3.1 I2C接口要求

需要实现以下I2C操作函数：

```c
// I2C写操作
int8_t icm42688_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len);

// I2C读操作  
int8_t icm42688_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len);
```

### 3.2 SPI接口要求

如果使用SPI接口，需要实现：

```c
// SPI写操作
int8_t icm42688_spi_write(uint8_t reg_addr, const uint8_t *data, uint32_t len);

// SPI读操作
int8_t icm42688_spi_read(uint8_t reg_addr, uint8_t *data, uint32_t len);

// CS引脚控制
void icm42688_cs_set(bool state);
```

### 3.3 延时函数

需要提供毫秒级延时函数：

```c
void icm42688_delay_ms(uint32_t ms);
```

## 4. API参考

### 4.1 初始化函数

- `icm42688_init()` - 初始化ICM42688P设备
- `icm42688_soft_reset()` - 软件复位设备
- `icm42688_set_power_mode()` - 设置电源模式

### 4.2 数据读取函数

- `icm42688_read_data()` - 读取加速度计和陀螺仪数据
- `icm42688_read_accel()` - 仅读取加速度计数据
- `icm42688_read_gyro()` - 仅读取陀螺仪数据

### 4.3 配置函数

- `icm42688_set_accel_fs()` - 设置加速度计量程
- `icm42688_set_gyro_fs()` - 设置陀螺仪量程
- `icm42688_set_odr()` - 设置输出数据率

## 5. 示例工程

完整的使用示例请参考 `demo/xmake` 目录中的工程文件。

## 6. 注意事项

1. **电源要求**：ICM42688P工作电压为1.71V-3.6V
2. **I2C地址**：AD0引脚接地时地址为0x68，接VDD时地址为0x69
3. **SPI模式**：支持SPI模式0和模式3
4. **数据单位**：加速度单位为g，角速度单位为dps（度/秒）
5. **温度补偿**：建议在关键应用中进行温度补偿校准