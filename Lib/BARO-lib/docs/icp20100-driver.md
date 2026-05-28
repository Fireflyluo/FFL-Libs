# ICP20100 气压计驱动说明

## 1. 器件简介

ICP20100 是一颗高精度气压 / 温度传感器，适用于高度计、气象站、室内定位等场景。

- **制造商**：InvenSense (TDK)
- **通信接口**：I2C
- **默认 I2C 地址**：`0x63`（AD0 接 GND）/ `0x64`（AD0 接 VCC）
- **芯片 ID**：`0x0C` 寄存器期望值 `0x63`

## 2. 目录与文件

| 路径 | 职责 |
|------|------|
| `drivers/icp20100/include/icp20100_reg.h` | 寄存器地址、位域结构体、枚举（OP Mode / Meas Mode / Power Mode / FIFO） |
| `drivers/icp20100/include/icp20100.h` | 对外 API 声明、设备结构体、配置结构体 |
| `drivers/icp20100/src/icp20100.c` | 驱动实现 |
| `include/baro_icp20100.h` | 聚合头文件 |

## 3. 初始化流程

`icp20100_init()` 执行顺序：

```
1. icp20100_probe()        — 读 DEVICE_ID (0x0C) 和 VERSION (0xD3)
2. icp20100_soft_reset()   — 写软复位命令
3. 写 DUMMY_INIT (0xEE)    — 写入初始化序列
4. 写 DUMMY_VALUE (0xF0)   — 写入配置值
5. 写 MODE_SELECT (0xC0)   — 设置 OP_MODE、MEAS_MODE、POWER_MODE、FIFO_MODE
```

默认配置（`g_icp20100_default_cfg`）：

| 参数 | 默认值 |
|------|--------|
| 工作模式 | OP_MODE0 |
| 测量模式 | 连续测量（`ICP20100_MEAS_MODE_CONTINUOUS`） |
| 电源模式 | 活跃（`ICP20100_POWER_MODE_ACTIVE`） |
| FIFO 模式 | 压力 + 温度（`ICP20100_FIFO_PRES_TEMP`） |

## 4. 关键配置项

### 工作模式 `icp20100_op_mode_t`

| 枚举值 | 说明 |
|--------|------|
| `ICP20100_OP_MODE0` ~ `ICP20100_OP_MODE4` | 不同精度 / 采样率配置 |

### 测量模式 `icp20100_meas_mode_t`

| 枚举值 | 含义 |
|--------|------|
| `ICP20100_MEAS_MODE_FORCED` | 强制触发测量 |
| `ICP20100_MEAS_MODE_CONTINUOUS` | 连续自动测量 |

### 电源模式 `icp20100_power_mode_t`

| 枚举值 | 含义 |
|--------|------|
| `ICP20100_POWER_MODE_NORMAL` | 正常模式 |
| `ICP20100_POWER_MODE_ACTIVE` | 活跃模式 |

### FIFO 模式 `icp20100_fifo_mode_t`

| 枚举值 | 含义 |
|--------|------|
| `ICP20100_FIFO_PRES_TEMP` | 压力 + 温度 |
| `ICP20100_FIFO_TEMP_ONLY` | 仅温度 |
| `ICP20100_FIFO_TEMP_PRES` | 温度 + 压力 |
| `ICP20100_FIFO_PRES_ONLY` | 仅压力 |

## 5. 读数接口

### 原始数据读取

```c
int icp20100_read_raw(icp20100_dev_t *dev, icp20100_raw_sample_t *raw);
```

- 从 FIFO 读取压力和温度原始值（24 位）
- `raw->pressure_raw`：压力原始值
- `raw->temperature_raw`：温度原始值

### 物理量读取

```c
int icp20100_read_sample(icp20100_dev_t *dev, icp20100_sample_t *sample);
```

- 调用 `read_raw` 获取原始值并换算
- `sample->pressure_kpa`：压力，单位 kPa
- `sample->temperature_c`：温度，单位 °C

### 底层寄存器访问

```c
int icp20100_read_reg(icp20100_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
int icp20100_write_reg(icp20100_dev_t *dev, uint8_t reg, const uint8_t *data, uint16_t len);
```

## 6. 错误码 / 状态

所有 API 返回 `int` 类型：**0 成功，负数失败**。

| 返回值 | 含义 |
|--------|------|
| `0` | 成功 |
| `-EIO` | 总线传输失败 |
| `-ENODEV` | 设备未识别（chip_id 不匹配） |
| `-EINVAL` | 参数无效 |

### FIFO_FILL 寄存器（0xC4）

| 位 | 名称 | 含义 |
|----|------|------|
| bit 4:0 | FIFO_LEVEL | FIFO 当前填充深度 |
| bit 5 | FIFO_FULL | FIFO 满标志 |
| bit 6 | FIFO_EMPTY | FIFO 空标志 |
| bit 7 | FIFO_FLUSH | FIFO 刷新（写 1 清空） |

## 7. 调试建议

1. **I2C 扫描**：先确认总线上能发现 `0x63` 或 `0x64` 地址
2. **Probe 验证**：调用 `icp20100_probe` 检查 chip_id 是否为 `0x63`，同时读取 version 字段
3. **初始化序列**：ICP20100 需要写入 DUMMY_INIT 和 DUMMY_VALUE 寄存器完成内部初始化，跳过此步骤会导致读数异常
4. **OP Mode 选择**：不同 OP_MODE 对应不同精度和采样率，根据应用场景选择
5. **微秒延时**：驱动需要 `delay_us` 回调，确保平台侧实现正确
