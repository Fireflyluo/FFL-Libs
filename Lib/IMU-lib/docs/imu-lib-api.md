# IMU-lib 接口定义说明

本文档面向库使用者，说明 `Lib/IMU-lib` 的公共接口、数据结构、返回值约定及典型调用方式。

---

## 1. 接口分层架构

```
┌─────────────────────────────────────────────────┐
│              应用层 (Application)                │
├─────────────────────────────────────────────────┤
│  芯片驱动层 (Chip Driver)                       │
│  imu_qmi8658a_*  /  imu_icm42688p_*            │
├─────────────────────────────────────────────────┤
│  公共类型层 (imu_types.h / imu_bus.h)           │
│  imu_sample_t / imu_raw_sample_t / imu_bus_ops_t│
├─────────────────────────────────────────────────┤
│  硬件总线 (I2C / SPI 由使用者实现)              │
└─────────────────────────────────────────────────┘
```

**公共通用层**：`imu_bus.h`、`imu_types.h`，定义总线抽象和通用数据类型，与具体芯片无关。

**芯片专属层**：`imu_qmi8658a.h`、`imu_icm42688p.h`，提供各芯片的初始化、配置和数据读取接口。

---

## 2. 公共类型与总线抽象

### 2.1 总线消息标志

定义于 `imu_bus.h`，用于构造 `imu_bus_msg_t.flags`：

| 宏                      | 含义           |
| ----------------------- | -------------- |
| `IMU_BUS_MSG_WRITE`     | 写操作 (bit 0) |
| `IMU_BUS_MSG_READ`      | 读操作 (bit 1) |
| `IMU_BUS_MSG_STOP`      | 发送 STOP (bit 2) |

### 2.2 总线消息结构

```c
typedef struct {
    uint8_t *buf;   // 数据缓冲区指针
    uint16_t len;   // 数据长度（字节）
    uint8_t flags;  // 消息标志（上述宏的组合）
} imu_bus_msg_t;
```

### 2.3 总线完成回调

```c
typedef void (*imu_bus_done_cb_t)(void *user, int status);
```

- `user`：使用者上下文指针
- `status`：0 表示成功，非 0 表示失败

### 2.4 总线操作接口

```c
typedef struct {
    int (*xfer)(void *ctx,
                const imu_bus_msg_t *msgs,
                uint8_t cnt,
                imu_bus_done_cb_t cb,
                void *user);
    int (*cancel)(void *ctx);
} imu_bus_ops_t;
```

使用者需实现 `xfer` 和 `cancel` 两个函数，并在设备实例中注入。

- `xfer`：执行一组总线消息传输，返回 0 表示成功
- `cancel`：取消当前传输，返回 0 表示成功

### 2.5 延时函数类型

```c
typedef void (*imu_delay_ms_fn)(void *ctx, uint32_t ms);
```

使用者需提供毫秒级延时函数，用于复位等待等场景。

### 2.6 采样数据类型

**原始采样** (`imu_types.h`)：

```c
typedef struct {
    int16_t accel[3];      // 加速度原始值 [X, Y, Z]
    int16_t gyro[3];       // 角速度原始值 [X, Y, Z]
    int16_t temperature;   // 温度原始值
} imu_raw_sample_t;
```

**物理量采样** (`imu_types.h`)：

```c
typedef struct {
    float accel_mps2[3];   // 加速度 [m/s²]
    float gyro_rads[3];    // 角速度 [rad/s]
    float temperature_c;   // 温度 [°C]
    uint32_t timestamp_ms; // 时间戳 [ms]
} imu_sample_t;
```

---

## 3. QMI8658A 接口

头文件：`imu_qmi8658a.h`

### 3.1 设备实例

```c
typedef struct {
    const imu_bus_ops_t *bus_ops;  // 总线操作函数表
    void *bus_ctx;                 // 总线上下文
    imu_delay_ms_fn delay_ms;      // 延时函数
    void *delay_ctx;               // 延时上下文
    uint8_t addr;                  // I2C 从机地址
    uint8_t chip_id;               // 芯片 ID（probe 后填充）
    bool initialized;              // 初始化标志
    imu_qmi8658a_cfg_t cfg;        // 当前配置
} imu_qmi8658a_t;
```

使用者声明此结构体变量并传入各 API，驱动不负责分配内存。

### 3.2 配置结构

```c
typedef struct {
    uint8_t addr;                      // I2C 从机地址
    qmi8658a_accel_fs_t accel_fs;      // 加速度计量程
    qmi8658a_accel_odr_t accel_odr;    // 加速度计 ODR
    qmi8658a_gyro_fs_t gyro_fs;        // 陀螺仪量程
    qmi8658a_gyro_odr_t gyro_odr;      // 陀螺仪 ODR
    bool enable_accel;                 // 使能加速度计
    bool enable_gyro;                  // 使能陀螺仪
    bool enable_auto_increment;        // 地址自增
    bool enable_sync_sample;           // 同步采样
    bool accel_lpf_enable;             // 加速度计 LPF 使能
    uint8_t accel_lpf_mode;            // 加速度计 LPF 模式
    bool gyro_lpf_enable;              // 陀螺仪 LPF 使能
    uint8_t gyro_lpf_mode;             // 陀螺仪 LPF 模式
} imu_qmi8658a_cfg_t;
```

**默认配置**：`g_imu_qmi8658a_default_cfg`（extern 常量）

**量程枚举**：

| 类型                    | 枚举值                                                                                                    |
| ----------------------- | --------------------------------------------------------------------------------------------------------- |
| `qmi8658a_accel_fs_t`   | `QMI8658A_ACCEL_FS_2G`(±2g) / `_4G` / `_8G` / `_16G`                                                      |
| `qmi8658a_gyro_fs_t`    | `QMI8658A_GYRO_FS_16DPS`(±16dps) ~ `_2048DPS`(±2048dps)                                                   |
| `qmi8658a_accel_odr_t`  | `QMI8658A_ACCEL_ODR_7174_4HZ_NORMAL` ~ `_3HZ_LP`（Normal / Low Power 模式）                               |
| `qmi8658a_gyro_odr_t`   | `QMI8658A_GYRO_ODR_7174_4HZ` ~ `_56_05HZ`                                                                 |

### 3.3 API 列表

| 函数                              | 说明                                          |
| --------------------------------- | --------------------------------------------- |
| `imu_qmi8658a_init(dev, cfg)`     | 初始化设备，写入配置寄存器                    |
| `imu_qmi8658a_probe(dev, who)`    | 读取 WHO_AM_I 寄存器，写入 `*who`             |
| `imu_qmi8658a_soft_reset(dev)`    | 软复位                                        |
| `imu_qmi8658a_read_reg(dev, reg, data, len)`  | 读寄存器（通用，底层）         |
| `imu_qmi8658a_write_reg(dev, reg, data, len)` | 写寄存器（通用，底层）         |
| `imu_qmi8658a_read_raw(dev, raw)` | 读原始采样                                    |
| `imu_qmi8658a_read_sample(dev, sample)` | 读物理量采样（自动换算）              |
| `imu_qmi8658a_set_accel_config(dev, fs, odr)` | 运行时修改加速度计配置       |
| `imu_qmi8658a_set_gyro_config(dev, fs, odr)`  | 运行时修改陀螺仪配置         |

---

## 4. ICM42688P 接口

头文件：`imu_icm42688p.h`

### 4.1 设备实例

```c
typedef struct {
    const imu_bus_ops_t *bus_ops;  // 总线操作函数表
    void *bus_ctx;                 // 总线上下文
    imu_delay_ms_fn delay_ms;      // 延时函数
    void *delay_ctx;               // 延时上下文
    uint8_t addr;                  // I2C 从机地址
    uint8_t chip_id;               // 芯片 ID
    uint8_t current_bank;          // 当前寄存器 Bank
    bool initialized;              // 初始化标志
    imu_icm42688p_cfg_t cfg;       // 当前配置
} imu_icm42688p_t;
```

### 4.2 配置结构

```c
typedef struct {
    uint8_t addr;                      // I2C 从机地址
    icm42688_sensor_mode_t accel_mode; // 加速度计工作模式
    icm42688_sensor_mode_t gyro_mode;  // 陀螺仪工作模式
    icm42688_accel_fs_t accel_fs;      // 加速度计量程
    icm42688_odr_t accel_odr;          // 加速度计 ODR
    icm42688_gyro_fs_t gyro_fs;        // 陀螺仪量程
    icm42688_odr_t gyro_odr;           // 陀螺仪 ODR
} imu_icm42688p_cfg_t;
```

**默认配置**：`g_imu_icm42688p_default_cfg`（extern 常量）

**关键枚举**：

| 类型                     | 说明                                                            |
| ------------------------ | --------------------------------------------------------------- |
| `icm42688_sensor_mode_t` | `ICM42688_MODE_OFF` / `MODE_STANDBY` / `MODE_LOW_POWER` / `MODE_LOW_NOISE` |
| `icm42688_accel_fs_t`    | `ICM42688_ACCEL_FS_16G`(±16g) / `_8G` / `_4G` / `_2G`          |
| `icm42688_gyro_fs_t`     | `ICM42688_GYRO_FS_2000DPS`(±2000dps) ~ `_15_625DPS`            |
| `icm42688_odr_t`         | `ICM42688_ODR_32000HZ` ~ `_1_5625HZ`（加速度计和陀螺仪共用）    |

### 4.3 API 列表

| 函数                                | 说明                                        |
| ----------------------------------- | ------------------------------------------- |
| `imu_icm42688p_init(dev, cfg)`      | 初始化设备                                  |
| `imu_icm42688p_probe(dev, who)`     | 读取 WHO_AM_I 寄存器                        |
| `imu_icm42688p_soft_reset(dev)`     | 软复位                                      |
| `imu_icm42688p_read_reg(dev, bank, reg, data, len)`  | 读指定 Bank 的寄存器      |
| `imu_icm42688p_write_reg(dev, bank, reg, data, len)` | 写指定 Bank 的寄存器      |
| `imu_icm42688p_read_raw(dev, raw)`  | 读原始采样                                  |
| `imu_icm42688p_read_sample(dev, sample)` | 读物理量采样（自动换算）              |
| `imu_icm42688p_set_accel_config(dev, fs, odr)` | 运行时修改加速度计配置       |
| `imu_icm42688p_set_gyro_config(dev, fs, odr)`  | 运行时修改陀螺仪配置         |

> **注意**：ICM42688P 的寄存器采用 Bank 机制，`read_reg`/`write_reg` 需指定 `icm42688_bank_t` 参数。`init`/`probe`/`read_raw`/`read_sample` 等高层 API 内部自动处理 Bank 切换，无需使用者关心。

---

## 5. 典型调用顺序

以 QMI8658A 为例（ICM42688P 同理）：

```c
#include "imu_qmi8658a.h"

// 1. 声明设备实例
imu_qmi8658a_t imu_dev;

// 2. 准备总线操作（由使用者实现）
static const imu_bus_ops_t my_bus_ops = {
    .xfer   = my_i2c_xfer,
    .cancel = my_i2c_cancel,
};

// 3. 填充设备实例的基础字段
imu_dev.bus_ops  = &my_bus_ops;
imu_dev.bus_ctx  = my_i2c_handle;
imu_dev.delay_ms = my_delay_ms;
imu_dev.delay_ctx = NULL;

// 4. 可选：使用默认配置
imu_qmi8658a_cfg_t cfg = g_imu_qmi8658a_default_cfg;
cfg.addr = 0x6A;  // 根据实际硬件设置地址

// 5. 初始化
int ret = imu_qmi8658a_init(&imu_dev, &cfg);
if (ret != 0) { /* 错误处理 */ }

// 6. 可选：验证芯片 ID
uint8_t who = 0;
ret = imu_qmi8658a_probe(&imu_dev, &who);
if (ret != 0 || who != QMI8658A_ID) { /* 错误处理 */ }

// 7. 读取数据
imu_sample_t sample;
ret = imu_qmi8658a_read_sample(&imu_dev, &sample);
if (ret == 0) {
    // 使用 sample.accel_mps2[0..2]
    // 使用 sample.gyro_rads[0..2]
    // 使用 sample.temperature_c
}

// 8. 可选：运行时修改量程/ODR
imu_qmi8658a_set_accel_config(&imu_dev, QMI8658A_ACCEL_FS_4G, QMI8658A_ACCEL_ODR_500HZ_ACC_ONLY);
```

---

## 6. 错误处理约定

- 所有 API 返回 `int` 类型：**0 表示成功，非 0 表示错误**
- 常见错误来源：
  - 总线传输失败（`xfer` 返回非 0）
  - 芯片 ID 校验不匹配
  - 设备未初始化时调用数据读取接口
- 使用者应在每次 API 调用后检查返回值
- `probe` 的 `who_am_i` 输出参数可用于二次确认芯片身份

---

## 7. 当前边界与限制

- **总线实现**：`imu_bus_ops_t` 的 `xfer` / `cancel` 由使用者在应用层实现，库不提供 I2C/SPI 驱动
- **内存管理**：驱动不进行动态内存分配，所有实例和缓冲区由使用者提供
- **FIFO**：当前公共 API 仅支持单次采样读取，FIFO 批量读取需通过 `read_reg` 直接操作寄存器
- **中断**：库不提供中断服务例程，中断引脚配置需使用者直接操作寄存器
- **平台适配**：`port/` 目录提供参考实现，新平台需自行移植总线和延时
- **包裁剪**：通过 xmake 的 `configs` 选择启用的驱动芯片，未启用的芯片不会编译进库
