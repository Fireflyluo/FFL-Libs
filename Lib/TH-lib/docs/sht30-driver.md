# SHT30 温湿度驱动说明

## 1. 器件简介

SHT30 是 Sensirion 数字温湿度传感器，通过 I2C 通信，提供高精度温度和相对湿度测量。

- **制造商**：Sensirion
- **通信接口**：I2C
- **默认 I2C 地址**：`0x44`（ADDR 接 GND）/ `0x45`（ADDR 接 VCC）

## 2. 目录与文件

| 路径 | 职责 |
|------|------|
| `drivers/sht30/include/sht30_core.h` | 核心类型（设备结构体、总线回调、配置类型、内部工具函数） |
| `drivers/sht30/include/sht30.h` | 对外 API 声明 |
| `drivers/sht30/src/sht30_core.c` | 基础工具函数 |
| `drivers/sht30/src/sht30_sync.c` | 同步接口实现 |
| `drivers/sht30/src/sht30_async.c` | 异步接口实现 |

## 3. 初始化流程

`sht30_init()` 执行顺序：

```
1. 验证设备结构体有效性
2. 获取原子锁（防止并发访问）
3. 若 addr == 0，设置默认地址 0x44
4. 清零异步上下文
5. 设置 initialized = true
6. 执行软复位（发送 0x30A2，等待 2ms）
7. 释放锁
```

## 4. 关键配置项

### 精度 / 重复性 `sht30_repeatability_t`

| 枚举值 | 含义 | 测量延时 |
|--------|------|----------|
| `SHT30_PRECISION_HIGH` | 高精度 | ~15ms |
| `SHT30_PRECISION_MEDIUM` | 中精度 | ~6ms |
| `SHT30_PRECISION_LOW` | 低精度 | ~4ms |

### 加热器控制 `sht30_heater_cmd_t`

| 枚举值 | 含义 |
|--------|------|
| `SHT30_HEATER_ENABLE` | 启用加热器（命令 `0x306D`） |
| `SHT30_HEATER_DISABLE` | 禁用加热器（命令 `0x3066`） |

## 5. 读数接口

### 同步读取

```c
int sht30_read_sample(sht30_dev_t *dev, sht30_repeatability_t repeatability, sht30_sample_t *out);
```

- 内部完成「发送测量命令 → 等待测量 → 读取 6 字节 → 解析」全过程
- `out->temperature_c`：温度，单位 °C
- `out->humidity_rh`：相对湿度，单位 %RH

### 数据格式

传感器返回 6 字节：`[T_MSB, T_LSB, T_CRC, H_MSB, H_LSB, H_CRC]`

换算公式：
```
temperature_c = -45.0 + 175.0 * (t_raw / 65535.0)
humidity_rh   = -6.0  + 125.0 * (h_raw / 65535.0)
```

湿度输出被钳位到 `[0.0, 100.0]` 范围。

### 状态寄存器读取

```c
int sht30_read_status(sht30_dev_t *dev, uint16_t *status);
```

### 加热器控制

```c
int sht30_heater(sht30_dev_t *dev, sht30_heater_cmd_t cmd);
```

### 异步接口

```c
int sht30_read_sample_async(sht30_dev_t *dev, sht30_repeatability_t repeatability,
                            sht30_sample_cb_t cb, void *user);
int sht30_soft_reset_async(sht30_dev_t *dev, sht30_done_cb_t cb, void *user);
int sht30_cancel_async(sht30_dev_t *dev);
```

同一设备同一时刻只能有一个异步操作（`in_use` 互斥），并发调用返回 `-EBUSY`。

## 6. 错误码 / 状态

所有 API 返回 `int` 类型：**0 成功，负数失败**。

| 返回值 | 含义 |
|--------|------|
| `0` | 成功 |
| `-EINVAL` | 参数无效 |
| `-ENODEV` | 设备未初始化 |
| `-EBUSY` | 设备正忙 |
| `-EIO` | 总线传输失败 |

## 7. 调试建议

1. **I2C 扫描**：确认总线上能发现 `0x44` 或 `0x45` 地址
2. **必须实现 `delay_ms`**：若为 NULL，测量命令发出后立即读取，会读到无效数据
3. **CRC 校验**：SHT30 返回的数据包含 CRC 校验字节，驱动内部会校验
4. **加热器自热**：长时间启用加热器会影响温湿度测量精度，测量前应停止加热
5. **与 SHT40 的区别**：SHT30 默认地址 `0x44`，SHT40 默认地址也是 `0x44`，但命令集不同
