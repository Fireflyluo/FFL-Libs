# 使用示例

`examples/` 放置可以复制、检查和改造的最小工程。示例不会代替正式组件文档，但能展示 Xmake target、组件依赖和 port 的组合方式。

## 当前示例

### driver-dual-entry

目录：[driver-dual-entry](driver-dual-entry/)

展示 SC7A20 的两种接入方式：

- `component/`：直接使用源码组件，适合裁剪和查看依赖。
- `quick-package/`：使用本地 `ffl-sc7a20` package，适合快速接入。
- `source-trimmed/`：展示裁剪后的源文件组织方式。

先阅读该目录的 README，再从对应子目录执行 Xmake 命令。示例使用 host 或交叉工具链时，以示例自己的 `xmake.lua` 为准。

### stm32-base-driver

目录：[stm32-base-driver](stm32-base-driver/)

STM32F103C8T6 最小系统板交叉编译固件示例，把 `ffl.osal`、`ffl.sc7a20`、
`ffl.ringbuffer`、`ffl.sw_timer` 四个组件和一个 HAL 板级工程接在一起：

- 展示 `ffl.driver_port` 南向能力在真实 MCU 上的写法：I2C transport、时间 ops、临界区；
- 展示 SysTick 单一时基同时喂给 HAL、OSAL 与 sw_timer 的接法；
- 组件源码来自仓库 `components/`（不复制），构建用 `-p cross --toolchain=arm-none-eabi`；
- 接线（I2C1: PB6/PB7、LED: PC13）、构建/烧录与验证现象写在示例 README 中。

## 自己创建示例

建议每个示例只回答一个问题，并在 README 中写明：

- 使用的组件 target；
- 使用的 MCU、板卡和工具链；
- 应用必须提供的 bus、GPIO、IRQ、时间或临界区回调；
- host/mock 与真实硬件验证的区别。

CherryUSB 等第三方库的可运行接入也放在这里，但上游源码仍保留在 [third_party/README.md](../third_party/README.md) 所述的位置。
