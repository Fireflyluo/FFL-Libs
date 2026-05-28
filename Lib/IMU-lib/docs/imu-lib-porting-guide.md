# IMU-lib 移植说明

面向将 `imu-lib` 移植到 PY32、CH32 等 MCU 平台的开发者。

## 1. 适用范围

本文档适用于：

- **目标平台**：PY32F4xx、CH32V2xx 等资源受限的 Cortex-M / RISC-V MCU
- **传输方式**：I2C 为主，SPI 也支持（通过同一套 `imu_bus_ops_t`）
- **传感器**：QMI8658A（默认地址 `0x6A`）、ICM42688P（默认地址 `0x69`）
- **构建系统**：xmake（通过 `add_requires` / `add_packages` 接入）

不适用于：不使用 xmake 构建的项目（需自行管理源文件和头文件路径）。

## 2. 目录结构与依赖边界

```
Lib/IMU-lib/
├── include/                    # 跨传感器通用接口
│   ├── imu_bus.h               # 总线抽象（imu_bus_ops_t、imu_bus_msg_t）
│   ├── imu_types.h             # 通用数据结构（imu_raw_sample_t、imu_sample_t）
│   ├── imu_qmi8658a.h          # QMI8658A 驱动对外 API
│   └── imu_icm42688p.h         # ICM42688P 驱动对外 API
├── drivers/
│   ├── qmi8658a/
│   │   ├── include/
│   │   │   └── qmi8658a_reg.h  # 寄存器地址 + 位域结构体 + 枚举
│   │   └── src/
│   │       └── imu_qmi8658a.c  # 驱动实现
│   └── icm42688p/
│       ├── include/
│       │   └── icm42688_reg.h
│       └── src/
│           └── imu_icm42688p.c
├── port/                       # 平台适配示例（空目录，放你的适配代码）
│   ├── py32f403/
│   └── ch32v208/
├── docs/                       # 文档
├── xmake.lua                   # 包构建脚本
└── README.md
```

### 依赖边界

驱动核心（`drivers/*/src/*.c`）**只依赖**以下头文件：

| 头文件 | 来源 | 内容 |
|--------|------|------|
| `imu_bus.h` | `include/` | 总线抽象结构体，**平台侧实现** |
| `imu_types.h` | `include/` | 数据结构定义，纯类型无平台依赖 |
| `imu_qmi8658a.h` | `include/` | QMI8658A 对外 API 声明 |
| `imu_icm42688p.h` | `include/` | ICM42688P 对外 API 声明 |
| `qmi8658a_reg.h` | `drivers/qmi8658a/include/` | 寄存器定义 |
| `icm42688_reg.h` | `drivers/icm42688p/include/` | 寄存器定义 |

驱动核心**不依赖**任何平台 HAL 头文件（如 `py32f4xx_hal.h`、`ch32v20x.h`）。

## 3. 平台侧最小适配项

移植时**必须由平台侧实现**以下内容，驱动核心不会帮你做：

### 3.1 `imu_bus_ops_t` — 总线传输回调

定义在 `imu_bus.h:22-29`：

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

你需要提供一个 `const imu_bus_ops_t` 实例，实现两个函数指针：

#### `xfer` — 核心传输函数

**调用约定**：

- `cnt == 1`：单条消息（纯读或纯写）
- `cnt == 2`：标准 I2C 寄存器读写模式
  - `msgs[0]`：写寄存器地址（`len == 1`，`flags & IMU_BUS_MSG_WRITE`）
  - `msgs[1]`：读数据或写数据（`flags & IMU_BUS_MSG_READ` 或 `IMU_BUS_MSG_WRITE`，带 `IMU_BUS_MSG_STOP`）
- 返回值：`0` 成功，负数 errno 表示失败（推荐 `-EIO`、`-ETIMEDOUT`、`-EBUSY`、`-EINVAL`）
- `cb` 和 `user`：当前驱动核心均传 `NULL`（同步模式），可忽略

**不要在 xfer 中做以下事情**：

- 不要自行拼接寄存器地址和数据到同一个 buffer（驱动已分拆为两条 msg）
- 不要假设 `cnt` 永远是 2（保留 `cnt == 1` 的处理）
- 不要引入 `HAL_Delay` 或 `osDelay`（延时由单独的 `delay_ms` 回调处理）

#### `cancel` — 取消传输

当前驱动未使用，可以返回 `0` 占位。

### 3.2 `imu_delay_ms_fn` — 毫秒延时回调

定义在 `imu_bus.h:31`：

```c
typedef void (*imu_delay_ms_fn)(void *ctx, uint32_t ms);
```

封装平台的毫秒延时函数（如 `HAL_Delay`、`__WFI` 循环等）。

### 3.3 设备实例结构体填充

每个驱动都有一个实例结构体，你需要在初始化前填充以下字段：

**QMI8658A**（`imu_qmi8658a_t`，定义在 `imu_qmi8658a.h:31-42`）：

```c
imu_qmi8658a_t dev = {
    .bus_ops  = &g_my_bus_ops,   // 你的总线操作实现
    .bus_ctx  = &my_bus_ctx,     // 传给 xfer 的第一个参数
    .delay_ms = my_delay_ms,     // 你的延时函数
    .delay_ctx = NULL,            // 传给 delay_ms 的第一个参数
};
```

**ICM42688P**（`imu_icm42688p_t`，定义在 `imu_icm42688p.h:25-37`）：

```c
imu_icm42688p_t dev = {
    .bus_ops  = &g_my_bus_ops,
    .bus_ctx  = &my_bus_ctx,
    .delay_ms = my_delay_ms,
    .delay_ctx = NULL,
};
```

**地址字段**（`addr`）不需要手动设置 —— `init` 函数会从 `cfg` 或默认配置中读取。

### 3.4 不要放进驱动核心的内容

| 内容 | 放哪里 |
|------|--------|
| HAL 头文件（`xxx_hal.h`） | 仅在 `port/` 或应用层包含 |
| I2C/SPI 外设初始化代码 | 板级 `board_init()` 或 CubeMX 生成 |
| GPIO / PinMux 配置 | 板级初始化 |
| RTOS 任务创建 / 调度 | 应用层 |
| `HAL_Delay` / `osDelay` 直接调用 | 封装到 `imu_delay_ms_fn` 回调 |
| 中断处理、DMA 回调 | 平台侧，不进驱动 |
| `qmi8658a_reg.h` 中的旧 API 函数声明（如 `qmi8658a_ctx_t`、`qmi8658a_i2c_read`） | 该文件保留了历史兼容代码，新驱动不使用，不要依赖它们 |

## 4. xmake 包接入方式

### 4.1 添加本地仓库

如果 `imu-lib` 在本地仓库中，在项目 `xmake.lua` 中：

```lua
add_repositories("my-libs D:/path/to/0.fireflyluo-Embedded-Libs-main/xmake-repo")
add_requires("imu-lib", {configs = {qmi8658a = true, icm42688p = false}})
add_packages("imu-lib")
```

### 4.2 裁剪配置

通过 configs 选择启用哪些驱动，未启用的驱动源文件不会被编译，节省 Flash：

```lua
-- 只启用 QMI8658A
add_requires("imu-lib", {configs = {qmi8658a = true, icm42688p = false}})

-- 只启用 ICM42688P
add_requires("imu-lib", {configs = {qmi8658a = false, icm42688p = true}})

-- 两个都启用
add_requires("imu-lib", {configs = {qmi8658a = true, icm42688p = true}})
```

### 4.3 条件编译宏

xmake 会自动定义以下宏（`public` 作用域，应用层可见）：

| 宏 | 含义 |
|----|------|
| `IMU_LIB_AVAILABLE` | imu-lib 包已接入 |
| `IMU_DRIVER_QMI8658A` | QMI8658A 驱动已启用 |
| `IMU_DRIVER_ICM42688P` | ICM42688P 驱动已启用 |

可以在应用层用 `#ifdef IMU_DRIVER_QMI8658A` 做条件编译。

## 5. 最小移植步骤

以 PY32 + QMI8658A 为例，从零到读到第一个样本：

### Step 1：实现总线适配层

创建 `imu_port_adapter.h` 和 `imu_port_adapter.c`，放在你的应用层或 `port/py32f403/` 目录。

**头文件**（参考 PY32 已有实现）：

```c
#ifndef IMU_PORT_ADAPTER_H
#define IMU_PORT_ADAPTER_H

#include "imu_bus.h"
#include "py32f4xx_hal.h"  // 替换为你的平台 HAL

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr;
    uint32_t timeout_ms;
} py32_imu_i2c_bus_ctx_t;

extern const imu_bus_ops_t g_py32_imu_i2c_bus_ops;
void py32_imu_delay_ms(void *ctx, uint32_t ms);

#endif
```

**源文件**要点：

- 实现 `xfer` 函数，处理 `cnt == 1`（纯读/写）和 `cnt == 2`（寄存器读写）
- 对于 `cnt == 2` 的读操作，使用 `HAL_I2C_Mem_Read`（或等效 API），将 `msgs[0].buf[0]` 作为寄存器地址
- 将 HAL 返回值映射为 errno 负数（`HAL_OK -> 0`，`HAL_TIMEOUT -> -ETIMEDOUT`，`HAL_BUSY -> -EBUSY`，`HAL_ERROR -> -EIO`）

### Step 2：在板级初始化中配置 I2C

确保你的 MCU 的 I2C 外设已初始化（CubeMX 或手写），且：

- I2C 时钟已使能
- GPIO 已配置为 I2C 功能（开漏、上拉）
- I2C 速率 ≤ 400kHz（QMI8658A 和 ICM42688P 均支持）

### Step 3：接入 xmake 包

在项目 `xmake.lua` 中：

```lua
add_repositories("my-libs D:/path/to/0.fireflyluo-Embedded-Libs-main/xmake-repo")
add_requires("imu-lib", {configs = {qmi8658a = true}})
add_packages("imu-lib")
```

### Step 4：编写初始化代码

```c
#include "imu_qmi8658a.h"
#include "imu_port_adapter.h"

static py32_imu_i2c_bus_ctx_t bus_ctx = {
    .hi2c = &hi2c2,       // 你的 I2C 句柄
    .dev_addr = 0x6A,     // QMI8658A 默认地址
    .timeout_ms = 100,
};

static imu_qmi8658a_t imu_dev = {
    .bus_ops = &g_py32_imu_i2c_bus_ops,
    .bus_ctx = &bus_ctx,
    .delay_ms = py32_imu_delay_ms,
    .delay_ctx = NULL,
};

void imu_example_init(void) {
    int rc = imu_qmi8658a_init(&imu_dev, NULL);  // NULL 使用默认配置
    if (rc != 0) {
        // 处理错误
    }
}
```

### Step 5：读取数据

```c
void imu_example_read(void) {
    imu_sample_t sample;
    int rc = imu_qmi8658a_read_sample(&imu_dev, &sample);
    if (rc == 0) {
        // sample.accel_mps2[0..2] — 加速度，单位 m/s²
        // sample.gyro_rads[0..2]  — 角速度，单位 rad/s
        // sample.temperature_c    — 温度，单位 °C
    }
}
```

## 6. 板级联调建议

### 6.1 最小联调步骤

1. **总线扫描**：先用 I2C 扫描确认能发现 `0x69` / `0x6A`，排除硬件问题
2. **Probe 验证**：调用 `imu_qmi8658a_probe` 检查 WHO_AM_I 是否返回 `0x05`
3. **Init 验证**：调用 `imu_qmi8658a_init`，检查返回值是否为 `0`
4. **单次读取**：调用 `imu_qmi8658a_read_raw`，检查原始值是否非零
5. **连续采样**：周期性调用 `imu_qmi8658a_read_sample`，观察数据是否随运动变化

### 6.2 调试输出模板

```
QMI8658A init rc=0 addr=0x6A id=0x05
QMI8658A seq=10 accel=[0.12, -9.78, 0.05] gyro=[0.01, -0.02, 0.00]
```

### 6.3 多传感器共存

如果同一 I2C 总线上挂多个 IMU，分别创建独立的 `imu_xxx_t` 实例和 `bus_ctx`，`dev_addr` 设为不同值即可。驱动内部通过 `bus_ctx` 隔离。

## 7. 常见移植坑

### 7.1 I2C 地址左移

HAL 的 `HAL_I2C_Master_Receive` / `HAL_I2C_Mem_Read` 通常需要 7-bit 地址左移 1 位（`dev_addr << 1`）。如果读回来全是 `0xFF` 或 `0x00`，先检查地址是否正确左移。

### 7.2 `cnt == 2` 读操作的实现

驱动期望的 I2C 时序是：`START - [ADDR+W] - [REG] - RESTART - [ADDR+R] - [DATA...] - STOP`。

在 STM32/PY32 HAL 中，这对应 `HAL_I2C_Mem_Read`，不是先 `Master_Transmit` 再 `Master_Receive`。如果用两次独立传输，传感器可能会在第一次 STOP 后释放总线上下文。

### 7.3 errno 定义缺失

部分 MCU 工具链（如 ARMCC）不定义 `EIO`、`ENODEV`、`EINVAL` 等 POSIX errno。驱动源码内部有 `#ifndef EIO` 保护，但你的适配层如果也要用 errno，需要自行定义或包含 `<errno.h>`。

### 7.4 `M_PI` 未定义

驱动内部有 `#ifndef M_PI` 保护，但如果你的应用层也用 `M_PI`，确保 `<math.h>` 已包含或自行定义。

### 7.5 ICM42688P 的 Bank 切换

ICM42688P 的寄存器分布在 4 个 bank 中（`ICM42688_BANK0` ~ `ICM42688_BANK4`）。驱动内部会自动切换 bank，但你不需要关心这个机制 —— 只需确保 `xfer` 能正确传输即可。

### 7.6 `qmi8658a_reg.h` 中的旧 API

`qmi8658a_reg.h` 中包含 `qmi8658a_ctx_t`、`qmi8658a_i2c_read` 等旧 API 声明和函数指针类型。这些是历史遗留，**新驱动不使用它们**。不要基于 `qmi8658a_ctx_t` 写代码，应该用 `imu_qmi8658a_t`。

### 7.7 `cancel` 函数

`imu_bus_ops_t.cancel` 当前未被驱动调用。如果你的平台不支持取消传输，直接返回 `0` 即可。未来如果驱动支持异步模式，该接口会被激活。

### 7.8 SPI 模式

如果使用 SPI，`xfer` 中的 `msgs` 仍然适用：

- `IMU_BUS_MSG_WRITE`：MOSI 方向
- `IMU_BUS_MSG_READ`：MISO 方向
- CS 片选由你的 `xfer` 实现自行管理（在第一条 msg 前拉低，最后一条 msg 的 STOP 后拉高）
