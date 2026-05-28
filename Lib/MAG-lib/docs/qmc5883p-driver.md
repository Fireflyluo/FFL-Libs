# QMC5883P 磁力计驱动说明

## 1. 器件简介

QMC5883P 是一颗三轴数字磁力计，提供地磁场 X/Y/Z 三轴分量测量，适用于电子罗盘、航向角计算等场景。

- **制造商**：QST Corporation
- **通信接口**：I2C
- **默认 I2C 地址**：`0x2C`
- **芯片 ID**：`0x00` 寄存器期望值 `0x80`

## 2. 目录与文件

| 路径 | 职责 |
|------|------|
| `drivers/qmc5883p/include/qmc5883p_reg.h` | 寄存器地址、位域结构体、枚举（ODR/OSR/Range/Mode） |
| `drivers/qmc5883p/include/qmc5883p.h` | 对外 API 声明、设备结构体、配置结构体 |
| `drivers/qmc5883p/src/qmc5883p.c` | 驱动实现 |
| `include/mag_qmc5883p.h` | 聚合头文件 |

## 3. 初始化流程

`qmc5883p_init()` 执行顺序：

```
1. qmc5883p_probe()      — 读 CHIP_ID (0x00)，校验 chip_id == 0x80
2. qmc5883p_soft_reset() — 写 CONTROL_2 (0x0B) SOFT_RST 位，等待复位完成
3. 写 CONTROL_1 (0x0A)   — 设置 MODE、ODR、OSR1、OSR2
4. 写 CONTROL_2 (0x0B)   — 设置 RANGE、SET_RESET_MODE
```

默认配置（`g_qmc5883p_default_cfg`）：

| 参数 | 默认值 |
|------|--------|
| 工作模式 | 连续测量（`QMC5883P_MODE_CONTINUOUS`） |
| ODR | 100 Hz |
| OSR1 | 8× |
| OSR2 | 1× |
| 量程 | ±30G |
| SET/RESET 模式 | 正常（SET + RESET） |

## 4. 关键配置项

### 工作模式 `qmc5883p_mode_t`

| 枚举值 | 含义 |
|--------|------|
| `QMC5883P_MODE_SUSPEND` | 挂起（低功耗） |
| `QMC5883P_MODE_NORMAL` | 单次测量后自动挂起 |
| `QMC5883P_MODE_SINGLE` | 单次触发测量 |
| `QMC5883P_MODE_CONTINUOUS` | 连续测量 |

### 输出数据率 `qmc5883p_odr_t`

| 枚举值 | 含义 |
|--------|------|
| `QMC5883P_ODR_10HZ` | 10 Hz |
| `QMC5883P_ODR_50HZ` | 50 Hz |
| `QMC5883P_ODR_100HZ` | 100 Hz |
| `QMC5883P_ODR_200HZ` | 200 Hz |

### 量程 `qmc5883p_range_t`

| 枚举值 | 含义 |
|--------|------|
| `QMC5883P_RANGE_2G` | ±2 Gauss |
| `QMC5883P_RANGE_8G` | ±8 Gauss |
| `QMC5883P_RANGE_12G` | ±12 Gauss |
| `QMC5883P_RANGE_30G` | ±30 Gauss |

### 过采样率

- **OSR1**（`qmc5883p_osr1_t`）：滤波过采样，8× / 4× / 2× / 1×
- **OSR2**（`qmc5883p_osr2_t`）：积分过采样，1× / 2× / 4× / 8×

## 5. 读数接口

### 原始数据读取

```c
int qmc5883p_read_raw(qmc5883p_dev_t *dev, qmc5883p_vec3i16_t *out);
```

- 从 `XOUT_L`（0x01）起始，连续读取 6 字节
- 小端序拼接为 `int16_t`（X、Y、Z 各 2 字节）

### 物理量读取

```c
int qmc5883p_read_ut(qmc5883p_dev_t *dev, qmc5883p_vec3f_t *out);
```

- 调用 `read_raw` 获取原始值
- 换算公式：`uT = raw × (range_gauss / 32768) × 100`
- 输出单位：微特斯拉（μT）

### 底层寄存器访问

```c
int qmc5883p_read_reg(qmc5883p_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
int qmc5883p_write_reg(qmc5883p_dev_t *dev, uint8_t reg, const uint8_t *data, uint16_t len);
```

## 6. 错误码 / 状态

所有 API 返回 `int` 类型：**0 成功，负数失败**。

| 返回值 | 含义 |
|--------|------|
| `0` | 成功 |
| `-EIO` | 总线传输失败 |
| `-ENODEV` | 设备未识别（chip_id 不匹配） |
| `-EINVAL` | 参数无效 |

### STATUS 寄存器（0x09）

| 位 | 名称 | 含义 |
|----|------|------|
| bit 0 | DRDY | 数据就绪（1 = 新数据可用） |
| bit 1 | OVFL | 数据溢出（1 = 数据被覆盖） |

## 7. 调试建议

1. **I2C 扫描**：先确认总线上能发现 `0x2C` 地址
2. **Probe 验证**：调用 `qmc5883p_probe` 检查 chip_id 是否为 `0x80`
3. **数据就绪**：读取 STATUS 寄存器 DRDY 位，确认数据已就绪后再读取
4. **SET/RESET 模式**：若磁场数据异常，检查 `set_reset_mode` 配置是否正确
5. **量程选择**：地磁场约 25~65 μT，±2G 量程足够；若周围有强磁铁，需增大量程
