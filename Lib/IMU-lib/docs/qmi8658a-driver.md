# QMI8658A 驱动说明文档

## 1. 芯片与驱动定位

QMI8658A 是一颗 6 轴 IMU（三轴加速度计 + 三轴陀螺仪），内置温度传感器、FIFO、运动检测（敲击/计步/任意运动/无运动/显著运动）等特性。

本驱动位于 `Lib/IMU-lib/drivers/qmi8658a/`，通过 `imu_bus_ops_t` 总线抽象层访问芯片，不耦合具体 MCU HAL。当前仅实现**寄存器轮询读取**模式，未使用中断或 FIFO 批量读取。

## 2. 目录与文件职责

| 路径 | 职责 |
|------|------|
| `drivers/qmi8658a/include/qmi8658a_reg.h` | 寄存器地址、位域结构体、枚举（ODR/FS/FIFO）、底层通信函数声明 |
| `drivers/qmi8658a/src/imu_qmi8658a.c` | 驱动核心实现：初始化、配置、原始/换算数据读取 |
| `include/imu_qmi8658a.h` | 对外 API 声明、配置结构体 `imu_qmi8658a_cfg_t`、设备结构体 `imu_qmi8658a_t` |
| `include/imu_bus.h` | 总线抽象：`imu_bus_ops_t`（xfer/cancel）、`imu_bus_msg_t` 消息格式 |
| `include/imu_types.h` | 通用数据类型：`imu_raw_sample_t`（int16 原始值）、`imu_sample_t`（float 换算值） |

## 3. 当前支持的能力

### 已实现

- **设备探测**：读取 WHO_AM_I（0x00）寄存器，期望值 `0x05`
- **软复位**：向 RESET（0x60）寄存器写入 `0xB0`，等待 20ms
- **加速度计配置**：量程（±2/4/8/16g）、ODR（12 种，含 Normal 和 Low Power 模式）
- **陀螺仪配置**：量程（±16~2048dps）、ODR（8 种，7174.4Hz~56.05Hz）
- **低通滤波器开关**：加速度计/陀螺仪独立使能，模式 0~3
- **同步采样**：CTRL7.syncSmpl 使能加速度计与陀螺仪同步
- **地址自增**：CTRL1.ADDR_AI 使能连续读取
- **原始数据读取**：一次性读取 14 字节（温度 + 加速度 + 陀螺仪），小端序
- **换算数据输出**：加速度 → m/s²，陀螺仪 → rad/s，温度 → °C

### 未实现 / 未覆盖

- FIFO 模式（旁路/FIFO/流）及水位中断
- 运动检测（敲击、计步、任意运动、无运动、显著运动）
- 自检（Self-Test）
- 中断引脚配置（INT1/INT2 映射、边沿/电平触发）
- 时间戳读取（虽然寄存器头文件已定义，驱动未使用）
- SPI 三线/四线接口选择
- 大小端切换
- 陀螺仪睡眠模式（CTRL7.gSN）
- CTRL9 命令接口（头文件声明了 `qmi8658a_ctrl9_command`，但实现文件未包含）

## 4. 初始化路径与关键寄存器

`imu_qmi8658a_init()` 执行顺序：

```
1. imu_qmi8658a_probe()     — 读 WHO_AM_I (0x00)，校验 chip_id == 0x05
2. imu_qmi8658a_soft_reset() — 写 RESET (0x60) = 0xB0，延时 20ms
3. 写 CTRL1 (0x02)           — 设置 ADDR_AI（地址自增）
4. 写 CTRL2 (0x03)           — 设置 aFS（量程）+ aODR（采样率）
5. 写 CTRL3 (0x04)           — 设置 gFS（量程）+ gODR（采样率）
6. 写 CTRL5 (0x06)           — 设置加速度计/陀螺仪低通滤波器
7. 写 CTRL7 (0x08)           — 使能 aEN/gEN，设置 syncSmpl
```

默认配置（`g_imu_qmi8658a_default_cfg`）：

| 参数 | 默认值 |
|------|--------|
| I2C 地址 | `0x6A` |
| 加速度计量程 | ±4g |
| 加速度计 ODR | 125 Hz（仅加速度计模式） |
| 陀螺仪量程 | ±512 dps |
| 陀螺仪 ODR | 112.1 Hz |
| 加速度计/陀螺仪 | 均使能 |
| 地址自增 | 使能 |
| 同步采样 | 使能 |
| 低通滤波器 | 禁用 |

## 5. 采样读取路径

`imu_qmi8658a_read_raw()`：

- 从 `OUT_TEMP_L`（0x33）起始，连续读取 14 字节
- 解析顺序：temp_L, temp_H, accelX_L, accelX_H, accelY_L, accelY_H, accelZ_L, accelZ_H, gyroX_L, gyroX_H, gyroY_L, gyroY_H, gyroZ_L, gyroZ_H
- 小端序拼接为 `int16_t`

`imu_qmi8658a_read_sample()`：

- 调用 `read_raw` 获取原始值
- 换算公式：
  - `accel_mps2[i] = raw_accel[i] × (full_scale_g / 32768) × 9.80665`
  - `gyro_rads[i] = raw_gyro[i] × (full_scale_dps / 32768) × (π / 180)`
  - `temperature_c = raw_temp / 256.0`
- `timestamp_ms` 当前固定输出 `0`（未接入时间戳源）

## 6. 对外接口摘要

### 设备生命周期

```c
int imu_qmi8658a_init(imu_qmi8658a_t *dev, const imu_qmi8658a_cfg_t *cfg);
// cfg 传 NULL 使用默认配置；内部完成探测、复位、配置、使能

int imu_qmi8658a_probe(imu_qmi8658a_t *dev, uint8_t *who_am_i);
// 仅读取 WHO_AM_I，不执行其他操作

int imu_qmi8658a_soft_reset(imu_qmi8658a_t *dev);
// 软复位，等待 20ms
```

### 运行时配置

```c
int imu_qmi8658a_set_accel_config(imu_qmi8658a_t *dev,
                                  qmi8658a_accel_fs_t fs,
                                  qmi8658a_accel_odr_t odr);

int imu_qmi8658a_set_gyro_config(imu_qmi8658a_t *dev,
                                 qmi8658a_gyro_fs_t fs,
                                 qmi8658a_gyro_odr_t odr);
```

### 数据读取

```c
int imu_qmi8658a_read_raw(imu_qmi8658a_t *dev, imu_raw_sample_t *raw);
// 返回 int16 原始值，无换算

int imu_qmi8658a_read_sample(imu_qmi8658a_t *dev, imu_sample_t *sample);
// 返回 float 换算值（SI 单位）
```

### 底层寄存器访问

```c
int imu_qmi8658a_read_reg(imu_qmi8658a_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
int imu_qmi8658a_write_reg(imu_qmi8658a_t *dev, uint8_t reg, const uint8_t *data, uint16_t len);
```

### 设备结构体要点

`imu_qmi8658a_t` 使用前需填充：

- `bus_ops`：指向 `imu_bus_ops_t`，必须实现 `xfer` 回调
- `bus_ctx`：传递给 `xfer` 的上下文指针
- `delay_ms`：可选，软复位等待用；传 NULL 则跳过延时
- `addr`：I2C 7 位地址（默认 `0x6A`）

## 7. 已知限制与注意事项

1. **无 FIFO 支持**：寄存器头文件已定义 FIFO 相关寄存器和枚举，但驱动未实现 FIFO 配置与读取。高频采样场景需自行扩展。

2. **无运动检测**：敲击、计步、任意/无/显著运动检测的寄存器位域已定义（CTRL8），但驱动未使用。

3. **无中断支持**：INT1/INT2 引脚配置、中断映射、数据就绪中断均未实现。当前采用轮询方式读取。

4. **时间戳未使用**：芯片提供 24 位硬件时间戳（0x30~0x32），`read_raw` 未读取该字段，`sample->timestamp_ms` 固定为 0。

5. **CTRL9 命令未实现**：头文件声明了 `qmi8658a_ctrl9_command`，但 `.c` 文件中无此函数实现。FIFO 请求/复位等依赖 CTRL9 的功能不可用。

6. **返回值约定**：成功返回 `0`，失败返回负数 errno（`-EINVAL`、`-ENODEV`、`-EIO`）。

7. **总线依赖**：驱动通过 `imu_bus_ops_t.xfer` 发送消息，调用方需自行实现 I2C/SPI 适配层。

8. **配置生效时机**：`set_accel_config` / `set_gyro_config` 可在 `init` 之后调用，立即写入寄存器生效，无需重新初始化。

9. **加速度计 Low Power ODR**：ODR 枚举中 `QMI8658A_ACCEL_ODR_*_LP` 为低功耗模式专用值，与 Normal 模式 ODR 不可混用。
