# ACC-lib 接口定义文档

本文档面向 **ACC-lib** 的使用者，描述公共 API、核心类型、总线抽象接口、典型接入顺序及错误处理约定。

> 当前仅包含 **SC7A20** 加速度计驱动。

---

## 1. 包与接口分层

```
┌─────────────────────────────────────────────────┐
│  应用层 (Application)                            │
│  #include "acc_sc7a20.h"                        │
├─────────────────────────────────────────────────┤
│  芯片驱动层 (SC7A20 Driver)                      │
│  sc7a20.h — 同步/异步公共 API                    │
├─────────────────────────────────────────────────┤
│  核心层 (Core)                                   │
│  sc7a20_core.h — 寄存器读写、解码、配置下发      │
├─────────────────────────────────────────────────┤
│  总线抽象层 (Bus Abstraction)                    │
│  sc7a20_bus_ops_t — 由使用者实现 xfer / cancel  │
└─────────────────────────────────────────────────┘
```

| 层级 | 头文件 | 定位 |
|------|--------|------|
| 用户入口 | `acc_sc7a20.h` | 聚合头文件，直接 include `sc7a20.h` |
| 芯片公共 API | `sc7a20.h` | init / deinit / 读写 / 异步接口 |
| 核心内部 | `sc7a20_core.h` | 设备结构体、总线回调、配置类型、核心辅助函数 |
| 寄存器定义 | `sc7a20_reg.h` | 寄存器地址、位域结构体、枚举（ODR / FS / FIFO） |

**总线抽象接口**（需使用者实现）：`sc7a20_bus_ops_t`  
**芯片专属接口**：`sc7a20.h` 中的全部函数及 `sc7a20_reg.h` 中的寄存器/枚举定义

---

## 2. 核心类型与回调

### 2.1 设备实例 `sc7a20_dev_t`

```c
typedef struct {
    const sc7a20_bus_ops_t *ops;  // 总线操作回调（必须由使用者填充）
    void *bus_ctx;                // 总线上下文（传给 xfer/cancel）
    uint8_t addr;                 // I2C 从机地址

    sc7a20_cfg_t cfg;             // 当前生效配置
    bool initialized;             // 初始化状态标志
    uint8_t who_am_i;             // 芯片 ID
    uint8_t endian_ble;           // 字节序
    float sensitivity_g_per_lsb;  // 灵敏度 (g/LSB)

    volatile uint8_t in_use;      // 异步操作占用锁
    sc7a20_async_ctx_t async;     // 异步上下文
} sc7a20_dev_t;
```

使用者须在调用 `sc7a20_init` 前填充 `ops`、`bus_ctx`、`addr` 三个字段；其余字段由驱动内部管理。

I2C 地址由硬件 SDO 引脚决定：

| 宏 | 值 | 条件 |
|----|----|------|
| `SC7A20_I2C_ADDR_L` | `0x18` | SDO 接 GND |
| `SC7A20_I2C_ADDR_H` | `0x19` | SDO 悬空或接 VCC |

### 2.2 总线操作 `sc7a20_bus_ops_t`

```c
typedef struct {
    int (*xfer)(void *ctx,
                const sc7a20_comm_msg_t *msgs,
                uint8_t cnt,
                sc7a20_bus_done_cb_t cb,
                void *user);
    int (*cancel)(void *ctx);
} sc7a20_bus_ops_t;
```

| 回调 | 说明 |
|------|------|
| `xfer` | 执行一组总线消息传输。`msgs` 为消息数组，`cnt` 为消息数。同步模式下 `cb` 为 `NULL`，函数阻塞返回结果码；异步模式下立即返回，完成后调用 `cb(user, status)`。 |
| `cancel` | 取消当前进行中的异步传输。若无操作可返回 0。 |

**总线消息标志**（`sc7a20_comm_msg_t.flags`）：

| 宏 | 值 | 含义 |
|----|----|------|
| `SC7A20_COMM_WRITE` | `1 << 0` | 写方向 |
| `SC7A20_COMM_READ`  | `1 << 1` | 读方向 |
| `SC7A20_COMM_STOP`  | `1 << 2` | 本次消息后发送 STOP 条件 |

### 2.3 设备配置 `sc7a20_cfg_t`

```c
typedef struct {
    sc7a20_accel_fs_t range;      // 量程
    sc7a20_accel_odr_t odr;       // 输出数据率
    bool axis_x_en;               // X 轴使能
    bool axis_y_en;               // Y 轴使能
    bool axis_z_en;               // Z 轴使能
    bool block_data_update;       // 块数据更新 (BDU)
    bool high_resolution;         // 高分辨率模式
    bool low_power;               // 低功耗模式
} sc7a20_cfg_t;
```

默认配置可通过全局常量 `g_sc7a20_default_cfg` 获取。

### 2.4 枚举类型

#### 量程 `sc7a20_accel_fs_t`

| 枚举值 | 含义 |
|--------|------|
| `SC7A20_ACCEL_FS_2G` | ±2g |
| `SC7A20_ACCEL_FS_4G` | ±4g |
| `SC7A20_ACCEL_FS_8G` | ±8g |
| `SC7A20_ACCEL_FS_16G` | ±16g |

#### 输出数据率 `sc7a20_accel_odr_t`

| 枚举值 | 含义 |
|--------|------|
| `SC7A20_ACCEL_ODR_POWER_DOWN` | 电源关断 |
| `SC7A20_ACCEL_ODR_1_56HZ` | 1.56 Hz |
| `SC7A20_ACCEL_ODR_12_5HZ` | 12.5 Hz |
| `SC7A20_ACCEL_ODR_25HZ` | 25 Hz |
| `SC7A20_ACCEL_ODR_50HZ` | 50 Hz |
| `SC7A20_ACCEL_ODR_100HZ` | 100 Hz |
| `SC7A20_ACCEL_ODR_200HZ` | 200 Hz |
| `SC7A20_ACCEL_ODR_400HZ` | 400 Hz |
| `SC7A20_ACCEL_ODR_800HZ` | 800 Hz |
| `SC7A20_ACCEL_ODR_1_48KHZ` | 1.48 kHz |
| `SC7A20_ACCEL_ODR_2_66KHZ` | 2.66 kHz |
| `SC7A20_ACCEL_ODR_4_434KHZ` | 4.434 kHz |

#### FIFO 模式 `sc7a20_fifo_mode_t`

| 枚举值 | 含义 |
|--------|------|
| `SC7A20_FIFO_BYPASS_MODE` | 旁路（不使用 FIFO） |
| `SC7A20_FIFO_FIFO_MODE` | FIFO 模式（满则丢弃新数据） |
| `SC7A20_FIFO_STREAM_MODE` | 流模式（满则丢弃最早数据） |
| `SC7A20_FIFO_TRIGGER_MODE` | 触发模式 |

> 注意：FIFO 模式枚举已定义但当前公共 API 未封装对应设置函数，如需使用请通过 `sc7a20_write_reg` 直接操作寄存器。

### 2.5 数据类型

```c
typedef struct { int16_t x, y, z; } sc7a20_vec3i16_t;  // 原始 ADC 值
typedef struct { float x, y, z; }   sc7a20_vec3f_t;     // 转换后 (g)
```

### 2.6 异步回调

```c
typedef void (*sc7a20_done_cb_t)(void *user, int status);
typedef void (*sc7a20_read_xyz_cb_t)(void *user, const sc7a20_vec3i16_t *xyz, int status);
```

| 回调 | 触发场景 |
|------|----------|
| `sc7a20_done_cb_t` | 寄存器读写异步操作完成 |
| `sc7a20_read_xyz_cb_t` | XYZ 异步读取完成，`xyz` 指向原始数据 |

---

## 3. SC7A20 API 列表

### 3.1 初始化与反初始化

| 函数 | 原型 | 说明 |
|------|------|------|
| `sc7a20_init` | `int sc7a20_init(sc7a20_dev_t *dev)` | 使用默认配置初始化 |
| `sc7a20_init_with_config` | `int sc7a20_init_with_config(sc7a20_dev_t *dev, const sc7a20_cfg_t *cfg)` | 使用自定义配置初始化 |
| `sc7a20_deinit` | `int sc7a20_deinit(sc7a20_dev_t *dev)` | 反初始化，进入掉电模式 |
| `sc7a20_soft_reset` | `int sc7a20_soft_reset(sc7a20_dev_t *dev)` | 软复位，寄存器恢复默认值 |

### 3.2 设备识别

| 函数 | 原型 | 说明 |
|------|------|------|
| `sc7a20_get_who_am_i` | `int sc7a20_get_who_am_i(sc7a20_dev_t *dev, uint8_t *who_am_i)` | 读取芯片 ID，SC7A20 预期值 `0x11` |

### 3.3 同步数据读取

| 函数 | 原型 | 说明 |
|------|------|------|
| `sc7a20_read_xyz_raw` | `int sc7a20_read_xyz_raw(sc7a20_dev_t *dev, sc7a20_vec3i16_t *out)` | 读取三轴原始 ADC 值 |
| `sc7a20_read_xyz_g` | `int sc7a20_read_xyz_g(sc7a20_dev_t *dev, sc7a20_vec3f_t *out)` | 读取三轴加速度 (单位 g) |

### 3.4 运行时配置

| 函数 | 原型 | 说明 |
|------|------|------|
| `sc7a20_set_range` | `int sc7a20_set_range(sc7a20_dev_t *dev, sc7a20_accel_fs_t range)` | 设置量程 |
| `sc7a20_set_odr` | `int sc7a20_set_odr(sc7a20_dev_t *dev, sc7a20_accel_odr_t odr)` | 设置输出数据率 |
| `sc7a20_set_axis_enable` | `int sc7a20_set_axis_enable(sc7a20_dev_t *dev, bool x_en, bool y_en, bool z_en)` | 设置各轴使能 |

### 3.5 原始寄存器访问

| 函数 | 原型 | 说明 |
|------|------|------|
| `sc7a20_read_reg` | `int sc7a20_read_reg(sc7a20_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len)` | 同步读寄存器 |
| `sc7a20_write_reg` | `int sc7a20_write_reg(sc7a20_dev_t *dev, uint8_t reg, const uint8_t *data, uint16_t len)` | 同步写寄存器 |

### 3.6 异步 API

| 函数 | 原型 | 说明 |
|------|------|------|
| `sc7a20_read_reg_async` | `int sc7a20_read_reg_async(sc7a20_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len, sc7a20_done_cb_t cb, void *user)` | 异步读寄存器 |
| `sc7a20_write_reg_async` | `int sc7a20_write_reg_async(sc7a20_dev_t *dev, uint8_t reg, const uint8_t *data, uint16_t len, sc7a20_done_cb_t cb, void *user)` | 异步写寄存器 |
| `sc7a20_read_xyz_raw_async` | `int sc7a20_read_xyz_raw_async(sc7a20_dev_t *dev, sc7a20_read_xyz_cb_t cb, void *user)` | 异步读取三轴原始值 |
| `sc7a20_cancel_async` | `int sc7a20_cancel_async(sc7a20_dev_t *dev)` | 取消当前异步操作 |

> 异步 API 依赖总线层的 `xfer` 回调支持异步模式（`cb != NULL`）。若总线层仅支持同步，则不应调用异步接口。

---

## 4. 典型接入顺序

### 4.1 同步模式（最常见）

```c
#include "acc_sc7a20.h"

/* 1. 实现总线 xfer（同步模式下 cb 参数为 NULL，阻塞返回） */
static int my_bus_xfer(void *ctx, const sc7a20_comm_msg_t *msgs, uint8_t cnt,
                       sc7a20_bus_done_cb_t cb, void *user) {
    /* 通过硬件 I2C 发送/接收 msgs 数组，同步返回 0 或错误码 */
    (void)cb; (void)user;
    // ...
    return 0;
}

static int my_bus_cancel(void *ctx) { return 0; }

static const sc7a20_bus_ops_t bus_ops = {
    .xfer   = my_bus_xfer,
    .cancel = my_bus_cancel,
};

/* 2. 填充设备实例 */
sc7a20_dev_t dev = {
    .ops     = &bus_ops,
    .bus_ctx = NULL,        /* 按需传入 I2C 句柄等 */
    .addr    = SC7A20_I2C_ADDR_L,
};

/* 3. 初始化 */
int ret = sc7a20_init(&dev);               /* 或 sc7a20_init_with_config(&dev, &cfg) */
if (ret != 0) { /* 错误处理 */ }

/* 4. 运行时调整（可选） */
sc7a20_set_range(&dev, SC7A20_ACCEL_FS_4G);
sc7a20_set_odr(&dev, SC7A20_ACCEL_ODR_100HZ);

/* 5. 读取数据 */
sc7a20_vec3f_t accel;
ret = sc7a20_read_xyz_g(&dev, &accel);
if (ret == 0) {
    /* accel.x, accel.y, accel.z 单位 g */
}

/* 6. 反初始化 */
sc7a20_deinit(&dev);
```

### 4.2 自定义配置初始化

```c
sc7a20_cfg_t cfg = {
    .range             = SC7A20_ACCEL_FS_8G,
    .odr               = SC7A20_ACCEL_ODR_200HZ,
    .axis_x_en         = true,
    .axis_y_en         = true,
    .axis_z_en         = true,
    .block_data_update = true,
    .high_resolution   = true,
    .low_power         = false,
};
sc7a20_init_with_config(&dev, &cfg);
```

### 4.3 芯片识别验证

```c
uint8_t id = 0;
sc7a20_get_who_am_i(&dev, &id);
if (id != SC7A20_CHIP_ID) { /* 芯片不存在或通信异常 */ }
```

---

## 5. 错误处理约定

所有公共 API 函数返回 `int` 类型状态码：

| 返回值 | 含义 |
|--------|------|
| `0` | 成功 |
| 负值 | 失败（具体值由总线层或核心层映射） |

### 返回值来源

- **总线层错误**：`xfer` 回调返回非零值，核心层通过 `sc7a20_core_map_bus_status()` 映射为统一错误码。
- **参数校验**：`sc7a20_core_validate_dev()` 检查 `dev` 指针、`ops` 指针、`initialized` 状态。
- **并发保护**：`sc7a20_core_try_lock()` / `sc7a20_core_unlock()` 管理异步操作互斥，设备忙时返回错误。

### 建议的检查策略

1. 每次 API 调用后检查返回值是否为 `0`。
2. 初始化后调用 `sc7a20_get_who_am_i` 验证芯片 ID 是否为 `SC7A20_CHIP_ID (0x11)`。
3. 异步回调中检查 `status` 参数。

---

## 6. 边界与限制

- **单实例并发**：同一 `sc7a20_dev_t` 实例在同一时刻只允许一个异步操作，`in_use` 字段用作自旋锁。
- **总线层职责**：I2C 时序、DMA、中断调度等均由使用者在 `xfer` 回调中实现，驱动层不关心具体硬件。
- **FIFO / 中断**：寄存器定义（`sc7a20_reg.h`）中已包含 FIFO 和中断相关枚举与位域，但当前公共 API 未封装对应高层函数。如需使用，请通过 `sc7a20_read_reg` / `sc7a20_write_reg` 直接操作寄存器。
- **芯片专属**：`sc7a20.h` / `sc7a20_core.h` / `sc7a20_reg.h` 中的所有类型与函数均为 SC7A20 专属，不可直接用于其他加速度计。
- **构建选项**：在 xmake 项目中需设置 `acc_sc7a20=y` 启用驱动编译，否则构建将报错。
- **C 语言标准**：库要求 C11 或以上。
