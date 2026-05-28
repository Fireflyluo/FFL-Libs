# TH-lib API 接口定义文档

本文档面向 TH-lib 库使用者，说明公共接口、类型定义、返回值约定及典型调用方式。

---

## 1. 包与接口分层

TH-lib 采用三层结构：

| 层级 | 头文件 | 职责 |
|------|--------|------|
| 公共入口 | `th_sht40.h` | 聚合头文件，应用直接包含此文件 |
| 驱动接口 | `sht40.h` | 对外 API 声明（同步/异步） |
| 核心类型 | `sht40_core.h` | 类型定义、回调、总线抽象、内部工具函数 |

应用层只需关注 `sht40.h` 中的函数和 `sht40_core.h` 中的类型。

---

## 2. 核心类型与回调

### 2.1 设备实例 `sht40_dev_t`

```c
typedef struct {
    const sht40_bus_ops_t *ops;      // 总线操作函数表（必须）
    void *bus_ctx;                    // 总线上下文（传递给 ops 回调）
    uint8_t addr;                     // I2C 地址，0 则使用默认 0x46

    sht40_delay_ms_fn delay_ms;      // 延迟函数（可选，NULL 则跳过延迟）
    void *delay_ctx;                  // 延迟上下文

    bool initialized;                 // 由 sht40_init 设置
    volatile uint8_t in_use;          // 内部互斥标志
    sht40_async_ctx_t async;          // 内部异步状态
} sht40_dev_t;
```

**使用者需填充字段**：`ops`、`bus_ctx`、`addr`（可选）、`delay_ms`（可选）、`delay_ctx`（可选）。

### 2.2 总线操作表 `sht40_bus_ops_t`

```c
typedef struct {
    int (*xfer)(void *ctx, const sht40_comm_msg_t *msgs, uint8_t cnt,
                sht40_bus_done_cb_t cb, void *user);
    int (*cancel)(void *ctx);
} sht40_bus_ops_t;
```

- `xfer`：执行 I2C 传输。同步模式下 `cb` 可为 NULL；异步模式下必须在传输完成时调用 `cb`。
- `cancel`：取消当前异步传输（仅异步模式需要）。

### 2.3 传输消息 `sht40_comm_msg_t`

```c
typedef struct {
    uint8_t *buf;    // 数据缓冲区
    uint16_t len;    // 数据长度
    uint8_t flags;   // SHT40_COMM_WRITE / SHT40_COMM_READ / SHT40_COMM_STOP 的组合
} sht40_comm_msg_t;
```

### 2.4 测量结果 `sht40_sample_t`

```c
typedef struct {
    float temperature_c;   // 温度，单位 °C
    float humidity_rh;     // 相对湿度，单位 %RH
} sht40_sample_t;
```

### 2.5 精度枚举 `sht40_precision_t`

| 值 | 含义 |
|----|------|
| `SHT40_PRECISION_HIGH` | 高精度（默认） |
| `SHT40_PRECISION_MEDIUM` | 中精度 |
| `SHT40_PRECISION_LOW` | 低精度 |

### 2.6 加热器命令 `sht40_heater_cmd_t`

| 值 | 功率 / 持续时间 |
|----|-----------------|
| `SHT40_HEATER_200MW_1S` | 200mW / 1s |
| `SHT40_HEATER_200MW_100MS` | 200mW / 100ms |
| `SHT40_HEATER_110MW_1S` | 110mW / 1s |
| `SHT40_HEATER_110MW_100MS` | 110mW / 100ms |
| `SHT40_HEATER_20MW_1S` | 20mW / 1s |
| `SHT40_HEATER_20MW_100MS` | 20mW / 100ms |

### 2.7 回调类型

```c
// 异步操作完成回调（用于 soft_reset_async）
typedef void (*sht40_done_cb_t)(void *user, int status);

// 异步采样完成回调（用于 read_sample_async）
// status==0 时 sample 有效；status!=0 时 sample 为 NULL
typedef void (*sht40_sample_cb_t)(void *user, const sht40_sample_t *sample, int status);
```

---

## 3. SHT40 API 列表

### 3.1 同步接口

| 函数 | 说明 |
|------|------|
| `sht40_init(dev)` | 初始化设备，设置默认地址，执行软复位 |
| `sht40_soft_reset(dev)` | 发送软复位命令，等待 2ms |
| `sht40_read_serial(dev, &serial)` | 读取 32 位序列号 |
| `sht40_read_sample(dev, precision, &out)` | 同步采集一次温湿度 |
| `sht40_heater(dev, cmd)` | 触发加热器（阻塞等待加热完成） |

**同步接口行为**：函数内部完成「发送命令 → 等待测量 → 读取结果」全过程，返回时结果已可用。等待期间调用 `delay_ms`（如果已配置）。

### 3.2 异步接口

| 函数 | 说明 |
|------|------|
| `sht40_soft_reset_async(dev, cb, user)` | 异步软复位，完成时调用 `cb` |
| `sht40_read_sample_async(dev, precision, cb, user)` | 异步采样，完成时调用 `cb` |
| `sht40_cancel_async(dev)` | 取消当前异步操作 |

**异步接口行为**：函数立即返回（返回 0 表示已提交）。实际 I2C 传输由 `ops->xfer` 的回调驱动，完成后通过用户回调通知。同一设备同一时刻只能有一个异步操作（`in_use` 互斥），并发调用返回 `-EBUSY`。

---

## 4. 典型接入顺序

### 4.1 初始化

```c
sht40_dev_t dev = {0};
dev.ops      = &my_bus_ops;    // 实现 xfer（cancel 可选）
dev.bus_ctx  = &my_i2c_handle;
dev.addr     = 0;              // 0 = 使用默认地址 0x46
dev.delay_ms = my_delay_ms;   // 可选
dev.delay_ctx = NULL;

int rc = sht40_init(&dev);    // 内部执行软复位
```

### 4.2 同步读取

```c
sht40_sample_t sample;
int rc = sht40_read_sample(&dev, SHT40_PRECISION_HIGH, &sample);
if (rc == 0) {
    // sample.temperature_c, sample.humidity_rh 可用
}
```

### 4.3 异步读取

```c
static void on_sample(void *user, const sht40_sample_t *sample, int status) {
    if (status == 0 && sample != NULL) {
        // 处理 sample
    }
}

int rc = sht40_read_sample_async(&dev, SHT40_PRECISION_HIGH, on_sample, NULL);
// rc == 0 表示已提交，等待回调
```

### 4.4 读取序列号

```c
uint32_t serial;
int rc = sht40_read_serial(&dev, &serial);
```

### 4.5 使用加热器

```c
int rc = sht40_heater(&dev, SHT40_HEATER_200MW_1S);
```

---

## 5. 错误处理约定

所有 API 返回 `int` 类型状态码：

| 返回值 | 含义 |
|--------|------|
| `0` | 成功 |
| `-EINVAL` | 参数无效（NULL 指针等） |
| `-ENODEV` | 设备未初始化（未调用 `sht40_init`） |
| `-EBUSY` | 设备正忙（异步操作进行中） |
| `-ENOTSUP` | 操作不支持（如 `cancel_async` 但未实现 `ops->cancel`） |
| `-EIO` | I/O 错误（总线传输失败） |
| `-ENACK` | NACK（设备未应答） |

**CRC 校验**：`sht40_read_sample` / `sht40_read_sample_async` 内部校验数据 CRC，校验失败返回 `-EBADMSG`。

---

## 6. 边界与限制

- **并发限制**：同一 `sht40_dev_t` 实例同一时刻只能有一个异步操作。同步接口之间也互斥（通过 `in_use` 标志）。
- **异步回调上下文**：回调在 `ops->xfer` 的完成回调中执行，通常为中断或主循环上下文，回调内不应执行耗时操作。
- **delay_ms**：可选，但不配置时同步接口的测量等待将被跳过，可能导致读取失败。异步接口同理。
- **I2C 地址**：默认 `0x46`，可通过 `dev.addr` 覆盖。
- **不支持 heater 异步**：当前仅提供同步加热器接口。
- **总线层实现**：`ops->xfer` 和 `ops->cancel` 由使用者在平台适配层实现，驱动本身不依赖具体平台。
