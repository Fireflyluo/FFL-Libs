# Components

`components/` 是本仓库可复用源码的唯一正式落点。目录按技术领域分类，但分类目录只是 Xmake 的 `includes()` 索引，**不是**需要一并编译的大集合；应用只依赖自己所需的 `ffl.<component>` 叶子 target。

## 文件树

```text
components/
├── foundation/
│   ├── atomic/                       # ffl.atomic，跨编译器字节锁封装
│   ├── driver-port/                  # ffl.driver_port，统一 transport/time/GPIO/IRQ 契约
│   ├── ringbuffer/                   # ffl.ringbuffer，include/ffl/ringbuffer.h，GCC/G++ host 测试
│   └── sw-timer/                     # ffl.sw_timer，include/ffl/sw_timer.h，GCC/G++ host 测试
├── algorithms/
│   └── impact-displacement/          # ffl.impact_displacement，纯算法 core，C/C++ host 测试
├── runtime/
│   ├── protothreads/                 # ffl.protothreads，纯头文件
│   └── osal/                         # ffl.osal，通用 core 与显式 port hooks
├── protocols/
│   └── adhoc/                        # ffl.adhoc，协议 core
└── drivers/
    ├── sensor/
    │   ├── accelerometer/sc7a20/     # ffl.sc7a20 object + ffl.sc7a20.full static
    │   ├── environmental/sht30/      # ffl.sht30
    │   ├── environmental/sht40/      # ffl.sht40，通用 I2C transport + CH32 port
    │   ├── environmental/icp20100/   # ffl.icp20100
    │   ├── magnetometer/qmc5883p/    # ffl.qmc5883p
    │   └── imu/common|qmi8658a|icm42688p/ # QMI/ICM 均提供同步 I2C 6DOF facade
    ├── display/                      # 预留通用显示驱动组件入口
    └── radio/                        # 预留通用射频驱动组件入口
```

OLED、SI24R1、XN297L 与 XL2400P 目前仍是 CH32 固定 HAL 实现，已经隔离到 `ports/ch32/legacy/`，尚未满足正式组件契约，因此不会声明误导性的 `ffl.*` core target。

## 叶子组件契约

正式组件通常遵循：

```text
components/<domain>/<component>/
  include/          # 公开 API；不包含板级 HAL
  src/              # 唯一 core 实现
  docs/             # API、配置、移植说明
  test/             # host 或 mock 测试（如适用）
  xmake.lua         # ffl.<component> target
```

| 目录 | 用途 | 边界 |
|---|---|---|
| `include/` | 对外类型与 API | 不含 `main.h`、固定 GPIO 或全局外设句柄。 |
| `src/` | 唯一实现来源 | 不复制旧库或应用业务流程。 |
| `docs/` | 配置、移植、状态说明 | 以当前实现为准，不保留过时接口。 |
| `test/` | host / mock 自动验证 | 不把必须接开发板的程序伪装为 host test。 |
| `experimental/` | 待评审、未整合工作区 | 默认不被根 Xmake 索引包含。 |

所有 C 公开头提供 `extern "C"` 边界，根工程使用 C11 + C++17。公共 API 命名与南向接口契约见 `docs/API_STYLE.md`；C++ 调用方不需要也不应复制 C core。

## 两种驱动入口

### 源码组件（长期主入口）

`object` target 适合资源受限 MCU：调用工程可以选择该组件自己的编译期功能，例如 SC7A20 的 `sc7a20_async`。裁剪发生在编译阶段，驱动稳定后也继续保留这一入口。SC7A20 的 `ffl.sc7a20` 同时包含统一 facade 和 core；关闭 async 时不编译异步源文件。

```lua
includes("components/drivers/sensor/accelerometer/sc7a20")

target("firmware")
    set_kind("binary")
    add_deps("ffl.sc7a20")
```

### 完整 static package（快速入口）

仅对明确需要双入口的单设备驱动维护 `ffl-<device>` package。该 package 与源码 target 共用同一套 `include/` 和 `src/`，不允许复制 core，也不允许把同类全部驱动打进一个包。

SC7A20 的完整入口为 `ffl.sc7a20.full`，始终编译同步、异步和 portable facade。现有 recipe 仍使用 `ffl-sc7a20`，因此组件暂保留同内容的兼容静态 target；它不是跨设备 aggregate。两种入口都要求最终工程提供 `ffl.driver_port` 的 I2C transport；完整 static 不代表绑定固定 MCU HAL。

```lua
includes("components/drivers/sensor/accelerometer/sc7a20")
target("firmware")
    set_kind("binary")
    add_deps("ffl.sc7a20.full")
```

需要 Flash/RAM 裁剪时使用 `ffl.sc7a20`，并按需设置 `sc7a20_async=false`；需要最快接入且不裁剪时使用 `ffl.sc7a20.full`。

## Port 与验证

通用组件通过回调或 `ports/` 注入 SPI、I2C、GPIO、DMA、IRQ、clock 与临界区。`ports/` 可使用 MCU HAL，但不能被 core 反向依赖。

未在硬件上验证的驱动只能写为“编译通过”或“mock 通过”。完整的板端回补项目、端口范围和验收条件见 `docs/HARDWARE_VALIDATION_BACKLOG.md`。
