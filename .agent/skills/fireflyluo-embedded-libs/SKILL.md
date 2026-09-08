---
name: fireflyluo-embedded-libs
description: Use when the user wants to REUSE components from this embedded C/C++ library repository (fireflyluo-Embedded-Libs / FFL-Libs): choosing an ffl.* component, adding it to a consumer Xmake project, implementing ffl.driver_port southbound interfaces, OSAL / sw-timer platform wiring, ARM-none-eabi or WCH RISC-V toolchains, cross-compiling MCU firmware, running host/mock tests, or adapting one of the STM32 / CH32 / PY32 examples to their own board.
metadata:
  short-description: Help users pick and integrate this repo's embedded components into their own firmware
---

# fireflyluo Embedded Libraries — 使用者指南

本 skill 是**使用者的助手**：帮你在自己的 MCU 固件工程里接入本仓库的组件，而不是修改仓库内部结构。只有在用户明确要求迁移组件、重构仓库、整理验证记录时，才转到文末“维护者边界”。

仓库是“组件优先”：源码按领域放在 `components/`，但**只按需引用叶子组件**，没有按传感器/射频/中间件聚合的大库。

## 1. 快速定位：目录各放什么

| 目录 | 使用者该怎么用 |
| --- | --- |
| `components/` | 正式源码组件，含 `include/`(头)、`src/`(实现)、`docs/`、`test/`、`xmake.lua` |
| `ports/` | 平台适配参考（CH32/PY32），可借鉴但通常要按你的板子改 |
| `examples/` | 最小可运行工程，**最值得先抄**：`stm32-base-driver`、`driver-dual-entry` |
| `toolchains/` | ARM/WCH 工具链定义与**获取/安装文档**（`README.md`） |
| `xmake-repo/` | 本地完整静态包（如 `ffl-sc7a20`）快速入口 |
| `docs/` | 文档导航；`docs/maintainer/` 是维护者资料（一般不用看） |
| `third_party/` | 上游第三方库（如 CherryUSB `v1.6.1`），**不声明 `ffl.*` target** |

## 2. 选组件

打开 `components/README.md` 的“按需求选择”表，或读目标组件目录下 `docs/` 和 `include/ffl/*.h`。常用叶子 target：

- 基础：`ffl.atomic`、`ffl.ringbuffer`、`ffl.sw_timer`、`ffl.driver_port`
- 运行时：`ffl.protothreads`（纯头文件）、`ffl.osal`（事件/消息/定时器/内存）
- 传感器：`ffl.sc7a20`(加速度)、`ffl.sht30`/`ffl.sht40`/`ffl.icp20100`(温湿/气压)、`ffl.qmc5883p`(磁力)、`ffl.qmi8658a`/`ffl.icm42688p`(IMU)
- 其它：`ffl.adhoc`(协议 core)、`ffl.impact_displacement`(算法)

每个驱动都**不绑定具体 MCU HAL**：I2C/SPI/GPIO/DMA/IRQ/时钟/临界区要由使用者工程或 port 提供（见第 4 节）。

## 3. 接入方式（三选一）

### A. 源码组件（推荐，可裁剪、能看实现）

在**你自己的工程** `xmake.lua` 里引入叶子目录并 `add_deps`：

```lua
includes("path/to/fireflyluo-Embedded-Libs/components/foundation/ringbuffer")
includes("path/to/fireflyluo-Embedded-Libs/components/drivers/sensor/accelerometer/sc7a20")

target("firmware")
    set_kind("binary")
    set_languages("c11", "cxx17")
    add_files("src/*.c", "board/*.c")
    add_deps("ffl.ringbuffer", "ffl.sc7a20")
```

### B. 完整静态包（快速接入、不裁剪）

- Xmake 包：`add_repositories(... xmake-repo)` + `add_requires("ffl-sc7a20")` + `add_packages("ffl-sc7a20")`（见 `xmake-repo/README.md`）
- 或组件同源 target `add_deps("ffl.sc7a20.full")`

### C. 裁剪功能

带 option 的组件可在源码方式下关闭特性，例如 SC7A20 关异步：

```lua
includes(".../sc7a20")
option("sc7a20_async") set_default(false)
target("firmware")
    ...
    add_deps("ffl.sc7a20")
```

> “完整包”不代表带固定 HAL/管脚；总线能力仍由你的工程给。

## 4. 提供南向能力（`ffl.driver_port`）

总线型驱动通过 `ffl/driver_port.h` 接收硬件能力（`ffl_transport_t` / `ffl_time_ops_t` / `ffl_gpio_t` / `ffl_irq_t`）。接入顺序：

1. 实现 `ffl_transport_ops_t.xfer`：把 `ffl_xfer_msg_t[]` 转成你的 HAL 事务；I2C endpoint 用 `ffl_endpoint_i2c7(addr7)`（7 位地址），SPI 用 `ffl_endpoint_spi()`。
   - 同步（`done==NULL`）须返回前完成；异步（`done!=NULL`）提交成功只回调一次，提交失败不得回调，排队的 port 要复制 endpoint/消息描述并保 buffer 有效。
2. 组装 `ffl_transport_t`（ops + 你的 ctx + 默认 endpoint）；SPI 片选用 `ffl_gpio_t` 单独管，别让 transport 和 GPIO 重复操作同一根线。
3. 提供 `ffl_time_ops_t`（至少 `delay_ms`）。
4. 设备对象清零 → 调驱动 `*_bind()` → `*_config_init()` → `*_init()`。

**最小可抄实现**：`examples/stm32-base-driver/bsp/src/` 下的 `sc7a20_i2c_transport.c`（HAL I2C 轮询）、`stm32_time_ops.c`（DWT 微秒）、`osal_port_stm32.c`（临界区）就是现成范本。

**OSAL 平台接线**（`ffl.osal`）：

- 临界区：`osal_port_set_critical_hooks(enter, exit)`（如 PRIMASK 关/开中断，可嵌套）。
- 时基：SysTick 每 1ms 调 `osal_update_timers()` 推进任务定时器/系统时钟（STM32 示例即如此）。

**sw-timer 平台接线**（`ffl.sw_timer`）：

- `ffl_sw_timer_set_lock_hooks(lock, unlock)` + `ffl_sw_timer_wheel_init(tick_ms)`
- SysTick 里调 `ffl_sw_timer_tick_isr()`；主循环/任务里调 `ffl_sw_timer_process()` 执行到期回调
- 单个 1ms SysTick 可以同时喂 HAL、OSAL、sw_timer 三套时基（见示例 `bsp/src/stm32f1xx_it.c`）

## 5. 工具链与构建

工具链安装包不入仓库，获取/安装/校验见 `toolchains/README.md`。

Host 验证（MinGW GCC）：

```powershell
xmake f -P . -p mingw -a x86_64 -m release
xmake -P .
xmake test -P .
```

ARM Cortex-M（`arm-none-eabi-*`）：

```powershell
xmake f -P <你的工程目录> -p cross -a arm --toolchain=arm-none-eabi --sdk="D:/path/to/arm-gnu-toolchain-.../bin/.."
xmake -P <你的工程目录>
```

WCH RISC-V（`riscv32-wch-elf-*`）：

```powershell
xmake f -P <你的工程目录> -p cross -a riscv --toolchain=wch-riscv --sdk="E:/path/to/RISC-V Embedded GCC15"
```

完整可运行命令照抄 `examples/stm32-base-driver/readme.md`。MCU 固件若要给整 target 统一加 `-mcpu/-mthumb` 等，注意在 `xmake.lua` 加 `set_policy("check.auto_ignore_flags", false)`，否则 xmake 会吞掉这些标志（已在该示例验证）。

## 6. 验证边界：分清三类证据

- `xmake test -P .`（host/mock）：证明纯逻辑、接口契约、C/C++ 头兼容、fake transport 行为。
- 交叉编译通过：只说明能编译/链接，**不等于**板上工作。
- 真实硬件：需要实际板子 + 总线波形/读数。驱动未接板前只能说“编译/mock 通过”。

驱动/组件的当前验证状态先看组件 `docs/README.md`，汇总在 `docs/maintainer/HARDWARE_VALIDATION_BACKLOG.md`。真实上板、烧录、串口操作必须由用户明确要求并提供目标板与接线后才做。

## 7. 借鉴对象（改动最小就能跑）

- **`examples/stm32-base-driver`**：STM32F103C8T6 最小板，把 `ffl.osal` + `ffl.sc7a20` + `ffl.ringbuffer` + `ffl.sw_timer` 和一块 HAL 板级工程接在一起；`readme.md` 写清了 I2C/LED 接线、构建、烧录与验证现象。想换板就把 `bsp/src/*` 的引脚/时钟/外设初始化改成你的板子。
- **`examples/driver-dual-entry`**：同一驱动“源码裁剪入口 vs 完整包入口”的对比。
- 组件自带的 `test/` 展示不带硬件的正确调用方式，`docs/` 提供 API 与移植说明。

## 8. 维护者边界（仅当用户明确要求改仓库内部时才启用）

改仓库内部（新增/迁移组件、动 `components/`、`ports/`、维护验证清单）时，遵守 `docs/maintainer/` 与各目录 README：`components/` 是唯一正式源码、分类目录只是索引、不引入聚合大库、core 不依赖板级、`ports/` 允许 HAL 但不得反向混入 core、验证结论如实标注（编译/mock/硬件）。不要把本地生成物（`build/`、`.xmake/`、`.tmp/`、日志、工具链压缩包）提交进 Git。

## 9. 交付前快速检查（使用者视角）

1. 用户工程里只 `includes`/依赖了需要的叶子 `ffl.*` target，没有带进整类驱动。
2. 南向 `ffl_transport_ops_t`/`ffl_time_ops_t` 的 endpoint、消息标志、buffer 生命周期符合 `driver_port.h` 契约。
3. 时钟/时基唯一且一致：HAL/OSAL/sw_timer 都对齐同一个 tick。
4. 交叉编译 target 的 arch/`-mthumb`、启动文件与链接脚本匹配目标 MCU；工具链用 `--sdk` 或 PATH 正确。
5. 给用户的结论明确区分“编译通过 / host-mock 通过 / 板上实测”，不夸大验证。
