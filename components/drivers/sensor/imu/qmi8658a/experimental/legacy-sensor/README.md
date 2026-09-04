# QMI8658A 6轴IMU驱动说明文档

## 1. 概述

QMI8658A是一款高性能6轴惯性测量单元（IMU），集成了3轴加速度计和3轴陀螺仪。该器件具有超低功耗特性，支持宽量程配置，适用于可穿戴设备、物联网终端、无人机等对功耗敏感的应用场景。

### 1.1 主要特性

- **加速度计量程**：±2g, ±4g, ±8g, ±16g
- **陀螺仪量程**：±32dps, ±64dps, ±128dps, ±256dps, ±512dps, ±1024dps, ±2048dps
- **超低功耗**：工作电流低至50μA，睡眠模式电流<1μA
- **高精度**：内置温度补偿，提供稳定的性能
- **通信接口**：I2C接口（本驱动主要支持I2C）
- **内置FIFO**：支持数据缓冲，降低主机处理器负载
- **多种工作模式**：正常模式、低功耗模式、待机模式

### 1.2 驱动架构

```
QMI8658A Driver
├── Core Layer (qmi8658a_driver.c/h) # 核心驱动实现
├── Register Layer (qmi8658a_reg.c/h) # 寄存器定义和操作
└── Examples                            # 使用示例（参考demo目录）
```

## 2. 配置选项

### 2.1 设备配置结构

```c
typedef struct {
    uint8_t i2c_addr;              // I2C地址 (0x6B)
    qmi8658_accel_fs_t accel_fs;   // 加速度计量程
    qmi8658_gyro_fs_t gyro_fs;     // 陀螺仪量程
    qmi8658_odr_t accel_odr;       // 加速度计输出数据率
    qmi8658_odr_t gyro_odr;        // 陀螺仪输出数据率
    bool enable_accel;             // 使能加速度计
    bool enable_gyro;              // 使能陀螺仪
    bool low_power_mode;           // 低功耗模式使能
} qmi8658_config_t;
```

### 2.2 基本使用流程

```c
#include "qmi8658a_driver.h"

// 1. 初始化配置
qmi8658_config_t config = {
    .i2c_addr = 0x6B,
    .accel_fs = QMI8658_ACCEL_FS_4G,
    .gyro_fs = QMI8658_GYRO_FS_512DPS,
    .accel_odr = QMI8658_ODR_100HZ,
    .gyro_odr = QMI8658_ODR_100HZ,
    .enable_accel = true,
    .enable_gyro = true,
    .low_power_mode = false
};

// 2. 初始化设备
if (qmi8658_init(&config) != QMI8658_OK) {
    // 初始化失败处理
}

// 3. 读取传感器数据
qmi8658_data_t sensor_data;
if (qmi8658_read_data(&sensor_data) == QMI8658_OK) {
    // 处理加速度和角速度数据
    float accel_y = sensor_data.accel_y;
    float gyro_x = sensor_data.gyro_x;
}
```

## 3. 平台适配

### 3.1 I2C接口要求

需要实现以下I2C操作函数：

```c
// I2C写操作
int8_t qmi8658_i2c_write(uint8_t reg_addr, const uint8_t *data, uint16_t len);

// I2C读操作
int8_t qmi8658_i2c_read(uint8_t reg_addr, uint8_t *data, uint16_t len);
```

### 3.2 延时函数

需要提供毫秒级延时函数：

```c
void qmi8658_delay_ms(uint16_t ms);
```

## 4. API参考

### 4.1 初始化函数

- `qmi8658_init()` - 初始化QMI8658A设备
- `qmi8658_soft_reset()` - 软件复位设备
- `qmi8658_set_power_mode()` - 设置电源模式

### 4.2 数据读取函数

- `qmi8658_read_data()` - 读取加速度计和陀螺仪数据
- `qmi8658_read_accel()` - 仅读取加速度计数据
- `qmi8658_read_gyro()` - 仅读取陀螺仪数据

### 4.3 配置函数

- `qmi8658_set_accel_fs()` - 设置加速度计量程
- `qmi8658_set_gyro_fs()` - 设置陀螺仪量程
- `qmi8658_set_odr()` - 设置输出数据率
- `qmi8658_enable_low_power()` - 启用/禁用低功耗模式

## 5. 示例工程

完整的使用示例请参考 `demo/xmake` 目录中的工程文件。

## 6. 注意事项

1. **电源要求**：QMI8658A工作电压为1.7V-3.6V
2. **I2C地址**：固定I2C地址为0x6B
3. **数据单位**：加速度单位为g，角速度单位为dps（度/秒）
4. **启动时间**：设备上电后需要适当的启动时间（建议>10ms）
5. **低功耗模式**：在低功耗模式下，某些功能可能受限
6. **温度影响**：虽然内置温度补偿，但在极端温度环境下仍需考虑校准
## Quick Include

```c
#include "qmi8658a_driver.h"
```
