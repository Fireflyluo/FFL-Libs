# SHT40 温湿度驱动说明

## 1. 驱动定位与目录

SHT40 是 Sensirion 数字温湿度传感器，通过 I2C 通信。本驱动提供同步和异步两种接口模式。

**目录结构**:

```
Lib/TH-lib/
├── drivers/sht40/
│   ├── include/
│   │   ├── sht40.h          # 对外接口声明
│   │   └── sht40_core.h     # 内部核心类型与函数
│   └── src/
│       ├── sht40_core.c     # 基础工具函数
│       ├── sht40_sync.c     # 同步接口实现
│       └── sht40_async.c    # 异步接口实现
└── xmake.lua                # 构建配置
```

**构建启用**: 在 xmake 中设置 `th_sht40=y`，会编译上述源文件并定义 `TH_DRIVER_SHT40` 宏。

---

## 2. 关键数据结构与总线抽象

### 2.1 设备结构 `sht40_dev_t`

```c
typedef struct {
    const sht40_bus_ops_t *ops;  // 总线操作函数表
    void *bus_ctx;               // 总线上下文（用户自定义）
    uint8_t addr;                // I2C 地址，默认 0x46

    sht40_delay_ms_fn delay_ms;  // 延时函数（可选，NULL 则跳过延时）
    void *delay_ctx;             // 延时上下文

    bool initialized;            // 初始化标志
    volatile uint8_t in_use;     // 并发锁（原子 CAS）
    sht40_async_ctx_t async;     // 异步上下文
} sht40_dev_t;
```

### 2.2 总线操作 `sht40_bus_ops_t`

```c
typedef struct {
    int (*xfer)(void *ctx, const sht40_comm_msg_t *msgs, uint8_t cnt,
                sht40_bus_done_cb_t cb, void *user);
    int (*cancel)(void *ctx);
} sht40_bus_ops_t;
```

- `xfer`: 传输函数，同步模式下 `cb` 传 `NULL`；异步模式下传完成回调
- `cancel`: 取消异步操作（异步模式必须实现，否则 `sht40_cancel_async` 返回 `-ENOTSUP`）

### 2.3 通信消息 `sht40_comm_msg_t`

```c
typedef struct {
    uint8_t *buf;    // 数据缓冲区
    uint16_t len;    // 数据长度
    uint8_t flags;   // SHT40_COMM_WRITE / SHT40_COMM_READ / SHT40_COMM_STOP
} sht40_comm_msg_t;
```

### 2.4 测量结果 `sht40_sample_t`

```c
typedef struct {
    float temperature_c;  // 温度 (°C)
    float humidity_rh;    // 相对湿度 (%RH)
} sht40_sample_t;
```

---

## 3. 初始化与测量路径

### 3.1 初始化 `sht40_init`

```c
int sht40_init(sht40_dev_t *dev);
```

**流程**:
1. 验证 `dev` 和 `ops->xfer` 非空
2. 原子锁获取（`in_use` CAS）
3. 若 `dev->addr == 0`，设置默认地址 `0x46`
4. 清零异步上下文
5. 设置 `initialized = true`
6. 执行软复位（发送 `0x94`，延时 2ms）
7. 释放锁

### 3.2 精度命令与测量延时

| 精度 | 命令字节 | 测量延时 |
|------|---------|---------|
| `SHT40_PRECISION_HIGH` | `0xFD` | 10ms |
| `SHT40_PRECISION_MEDIUM` | `0xF6` | 6ms |
| `SHT40_PRECISION_LOW` | `0xE0` | 3ms |

### 3.3 加热器命令

| 命令 | 字节 | 延时 |
|------|------|------|
| `SHT40_HEATER_200MW_1S` | `0x39` | 1200ms |
| `SHT40_HEATER_200MW_100MS` | `0x32` | 150ms |
| `SHT40_HEATER_110MW_1S` | `0x2F` | 1200ms |
| `SHT40_HEATER_110MW_100MS` | `0x24` | 150ms |
| `SHT40_HEATER_20MW_1S` | `0x1E` | 1200ms |
| `SHT40_HEATER_20MW_100MS` | `0x15` | 150ms |

### 3.4 同步测量 `sht40_read_sample`

```c
int sht40_read_sample(sht40_dev_t *dev, sht40_precision_t precision, sht40_sample_t *out);
```

**流程**:
1. 验证参数，检查 `initialized`
2. 获取原子锁
3. 发送精度命令（1 字节写）
4. 调用 `delay_ms` 等待测量完成
5. 读取 6 字节响应
6. 释放锁
7. 解析数据并输出

---

## 4. 数据解析与物理量换算

### 4.1 原始数据格式

传感器返回 6 字节：`[T_MSB, T_LSB, T_CRC, H_MSB, H_LSB, H_CRC]`

- 温度原始值：`t_raw = (T_MSB << 8) | T_LSB`
- 湿度原始值：`h_raw = (H_MSB << 8) | H_LSB`
- CRC 校验字节：当前驱动**未实现 CRC 校验**

### 4.2 换算公式

```c
temperature_c = -45.0f + 175.0f * (t_raw / 65535.0f)
humidity_rh   = -6.0f  + 125.0f * (h_raw / 65535.0f)
```

湿度输出被钳位到 `[0.0, 100.0]` 范围。

### 4.3 序列号读取 `sht40_read_serial`

```c
int sht40_read_serial(sht40_dev_t *dev, uint32_t *serial);
```

- 命令：`0x89`，延时 1ms
- 读取 6 字节，拼接：`serial = (rx[0]<<24) | (rx[1]<<16) | (rx[3]<<8) | rx[4]`
- 注意：跳过了 `rx[2]` 和 `rx[5]`（CRC 字节）

---

## 5. 异步接口模型

### 5.1 异步函数

| 函数 | 用途 |
|------|------|
| `sht40_read_sample_async` | 异步读取温湿度 |
| `sht40_soft_reset_async` | 异步软复位 |
| `sht40_cancel_async` | 取消当前异步操作 |

### 5.2 异步流程（以 `sht40_read_sample_async` 为例）

```
调用者 → sht40_read_sample_async(dev, precision, cb, user)
  │
  ├─ 获取原子锁（失败返回 -EBUSY）
  ├─ 发送精度命令（异步 xfer，回调 = sht40_async_cmd_done）
  │
  └─ [总线完成回调] sht40_async_cmd_done
       ├─ 调用 delay_ms 等待测量完成
       ├─ 发起 6 字节异步读取（回调 = sht40_async_read_done）
       │
       └─ [总线完成回调] sht40_async_read_done
            ├─ 解析数据
            ├─ 释放原子锁
            └─ 调用用户回调 cb(user, &sample, status)
```

### 5.3 并发控制

- 使用 `__sync_bool_compare_and_swap` 实现无锁互斥
- `in_use == 0` 表示空闲，`in_use == 1` 表示忙
- 同步接口也使用同一把锁，确保同步/异步互斥
- `sht40_cancel_async` 调用 `ops->cancel` 后释放锁

### 5.4 回调签名

```c
typedef void (*sht40_done_cb_t)(void *user, int status);
typedef void (*sht40_sample_cb_t)(void *user, const sht40_sample_t *sample, int status);
```

- `status == 0` 成功，`sample` 有效
- `status != 0` 失败，`sample` 为 `NULL`

---

## 6. 已知限制与注意事项

### 6.1 已实现能力

- [x] 同步初始化、软复位、读取序列号
- [x] 同步三种精度测量（高/中/低）
- [x] 同步加热器控制（6 种模式）
- [x] 异步测量读取（带回调）
- [x] 异步软复位
- [x] 异步取消操作
- [x] 原子锁防并发
- [x] 可选延时函数（`delay_ms` 为 NULL 时跳过）

### 6.2 未实现/未覆盖能力

- [ ] **CRC 校验**：读取的数据未验证 CRC，可能接受错误数据
- [ ] **异步加热器控制**：`sht40_heater` 仅支持同步模式
- [ ] **异步读取序列号**：`sht40_read_serial` 仅支持同步模式
- [ ] **超时机制**：无操作超时保护，依赖总线层实现
- [ ] **错误重试**：通信失败后不自动重试
- [ ] **多设备实例锁**：`in_use` 是设备级锁，不同设备间无互斥
- [ ] **低功耗模式**：无 sleep/wakeup 支持
- [ ] **I2C 总线恢复**：无总线死锁恢复机制

### 6.3 使用注意

1. **必须实现 `delay_ms`**：若为 NULL，测量命令发出后立即读取，会读到无效数据
2. **总线 `xfer` 返回值约定**：成功返回 0，正数表示总线错误码（会被映射为 `-EIO`），负数直接透传
3. **异步模式下 `xfer` 必须支持回调**：同步模式可忽略 `cb` 参数
4. **`addr` 为 0 时自动设为 `0x46`**：仅在 `sht40_init` 中生效
5. **加热器长时间工作会自热**：影响温湿度测量精度，测量前应停止加热
