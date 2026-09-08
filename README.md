# fireflyluo Embedded Libraries

面向资源受限 MCU 的可裁剪 C/C++ 组件。你可以直接选择一个叶子组件接入现有固件，也可以使用仓库提供的 port、示例和本地 package。

## 仓库概览

本仓库面向需要在 MCU 固件中复用基础设施、运行时、协议、算法和传感器驱动的开发者。组件以 C API 为主，兼容 C++ 调用；默认构建基线为 C11 + C++17，构建系统使用 Xmake。

仓库按“通用代码”和“平台代码”分层：

| 目录           | 你可以从这里获得什么                                                   |
| -------------- | ---------------------------------------------------------------------- |
| `components/`  | 可独立接入的正式源码组件，例如环形缓冲、OSAL、协议、算法和传感器驱动。 |
| `ports/`       | MCU、开发板、总线、GPIO、IRQ、DMA、时钟和临界区适配。                  |
| `examples/`    | 展示组件组合方式的最小工程，可作为应用工程起点。                       |
| `tests/`       | host、mock 和跨组件测试说明及测试入口。                                |
| `third_party/` | 保持上游目录和许可证的第三方库，例如 CherryUSB `v1.6.1`。              |
| `xmake-repo/`  | 少量需要快速接入的本地完整 package。                                   |
| `docs/`        | 使用文档导航；仓库结构和验证维护资料位于 `docs/maintainer/`。          |

组件 core 不绑定具体 MCU HAL，也不自动创建按“传感器”“射频”或“中间件”聚合的大库。应用只选择需要的叶子 target，再由自己的工程或 `ports/` 提供硬件能力。这样既能减少无关代码，也能让同一套驱动逻辑适配不同 MCU 和板卡。

当前仓库同时包含三种成熟度不同的内容：正式组件、已有但仍待真实板端验证的 port，以及 `experimental/` 或 `legacy/` 下的实验和历史板级实现。编译通过或 mock 测试通过不等于真实硬件验证，具体范围见 [tests/README.md](tests/README.md) 和维护者的 [硬件验证清单](docs/maintainer/HARDWARE_VALIDATION_BACKLOG.md)。

## 快速开始

### 1. 准备环境

- Windows + PowerShell
- Xmake 2.5.9 或更高版本
- Host 验证：MinGW-w64 GCC
- ARM 目标：`arm-none-eabi` GNU Toolchain
- WCH RISC-V 目标：WCH RISC-V Embedded GCC

工具链选择和完整命令见 [toolchains/README.md](toolchains/README.md)。

### 2. 运行 host 验证

```powershell
xmake f -P . -p mingw -a x86_64 -m release
xmake -P .
xmake test -P .
```

### 3. 接入一个组件

在应用工程的 `xmake.lua` 中只引入所需叶子目录：

```lua
includes("path/to/fireflyluo-Embedded-Libs/components/drivers/sensor/accelerometer/sc7a20")

target("firmware")
    set_kind("binary")
    add_files("src/*.c", "board/*.c")
    add_deps("ffl.sc7a20")
```

驱动 core 不包含 MCU HAL。I2C、SPI、GPIO、IRQ、DMA、时钟和临界区由你的工程或 [ports/README.md](ports/README.md) 中的适配层提供。

## 驱动南向接口接入

驱动组件通过 `ffl.driver_port` 接收硬件能力，不直接调用厂商 HAL。接入一个总线型驱动时按以下顺序进行：

1. 实现 `ffl_transport_ops_t`：在 `xfer()` 中把 `ffl_xfer_msg_t` 转换为 MCU 的 I2C/SPI 事务；I2C 使用 `ffl_endpoint_i2c7(addr7)`，SPI 使用 `ffl_endpoint_spi()`。
2. 组装 `ffl_transport_t`：填入 transport ops、硬件上下文 `ctx` 和默认 endpoint。SPI 的 CS 不属于 transport，要单独通过 `ffl_gpio_t` 管理。
3. 实现驱动需要的 `ffl_time_ops_t`：通常至少提供 `delay_ms`；异步驱动还应提供能满足组件要求的时间源或调度能力。
4. 如果设备有 CS、CE、RESET 或其他控制线，使用 `ffl_gpio_t`；如果设备有数据中断，使用 `ffl_irq_t`，并在最终工程的 ISR/任务中显式调用驱动的 `on_irq` API。
5. 将设备对象清零，调用具体驱动的 `*_bind()`，再调用 `*_config_init()` 和 `*_init()`。不要在 bind 之前操作设备，也不要在异步传输尚未结束时复用消息缓冲区。

以 SHT40 为例，应用工程需要提供 I2C transport 和毫秒延时，然后绑定并初始化设备：

```c
#include <string.h>

#include "ffl/sht40.h"

static ffl_sht40_device_t sensor;
static ffl_transport_t i2c1_transport;
static ffl_time_ops_t board_time_ops;

void app_sht40_init(void)
{
    ffl_sht40_config_t config;

    memset(&sensor, 0, sizeof(sensor));
    ffl_sht40_config_init(&config);
    config.i2c_addr7 = 0x46u;

    /* i2c1_transport 和 board_time_ops 由板级代码提前填好。 */
    if (ffl_sht40_bind(&sensor, &i2c1_transport, &board_time_ops, 0) != 0) {
        return;
    }
    (void)ffl_sht40_init(&sensor, &config);
}
```

`xfer()` 的同步调用必须在返回前完成事务；带 `done` 的调用表示异步提交，成功提交后必须且只能回调一次，提交失败则不得回调。排队的 port 必须复制 endpoint 和消息描述，调用方要保持消息 buffer 有效直到同步返回或异步回调完成。完整类型和回调约束见 [`ffl/driver_port.h`](components/foundation/driver-port/include/ffl/driver_port.h) 及 [driver-port 文档](components/foundation/driver-port/docs/README.md)。

## 按任务选择入口

| 你要做什么              | 阅读                                           |
| ----------------------- | ---------------------------------------------- |
| 选择组件和 target       | [components/README.md](components/README.md)   |
| 找一个最小可运行工程    | [examples/README.md](examples/README.md)       |
| 接入 MCU、板卡或 HAL    | [ports/README.md](ports/README.md)             |
| 使用 ARM / WCH GCC      | [toolchains/README.md](toolchains/README.md)   |
| 使用 CherryUSB 等上游库 | [third_party/README.md](third_party/README.md) |
| 查看测试边界            | [tests/README.md](tests/README.md)             |
| 使用本地完整 package    | [xmake-repo/README.md](xmake-repo/README.md)   |

当前正式组件包括基础设施、运行时、协议、算法和多种传感器驱动；完整列表与 target 见 [components/README.md](components/README.md)。CherryUSB 以第三方 submodule 形式固定在 `v1.6.1`，不作为本仓库的 `ffl.*` 组件。

维护者资料集中在 [docs/maintainer/README.md](docs/maintainer/README.md)。
