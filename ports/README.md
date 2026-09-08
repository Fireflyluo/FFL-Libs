# 平台适配

`ports/` 是组件与 MCU、开发板、厂商 HAL 之间的连接层。组件 core 负责通用逻辑，port 负责把真实硬件能力转换成 core 需要的接口。

## 当前可用目录

| 目标             | 目录                        | 用途                                |
| ---------------- | --------------------------- | ----------------------------------- |
| CH32 Ad-Hoc 链路 | [ch32/adhoc](ch32/adhoc/)   | 将协议 core 接到 CH32 链路。        |
| CH32 SC7A20      | [ch32/sc7a20](ch32/sc7a20/) | 提供 SC7A20 的 CH32 I2C 等适配。    |
| CH32 SHT40       | [ch32/sht40](ch32/sht40/)   | 提供 `ffl_transport_t` 和时间适配。 |
| PY32 OSAL        | [py32/osal](py32/osal/)     | 提供临界区、tick 等 OSAL 能力。     |

`ch32/legacy/` 下的 OLED、SI24R1、XN297L 和 XL2400P 仍直接依赖固定 HAL 和板级资源，不是通用组件 port。

## 接入已有 port

先检查 port 的 README、头文件和 `xmake.lua`，确认它需要的 `board.h`、`drv_i2c.h`、时钟初始化或全局句柄。port 通常需要由最终固件 target 显式加入：

```lua
includes("path/to/components/drivers/sensor/environmental/sht40")
add_includedirs("path/to/ports/ch32/sht40/include")
add_files("path/to/ports/ch32/sht40/src/*.c")
```

具体文件名以目标 port 的实际目录为准。接入前还要完成 MCU 时钟、GPIO 复用、I2C/SPI、DMA 和 IRQ 初始化。

## 自己编写 port

只实现组件实际需要的能力：transport、时间、GPIO、IRQ 或临界区。不要把厂商 HAL 头文件带入 `components/`，也不要在 core 中保存板级全局句柄。完成 host/mock 验证后，再按目标板记录硬件证据。

硬件验证清单位于 [docs/maintainer/HARDWARE_VALIDATION_BACKLOG.md](../docs/maintainer/HARDWARE_VALIDATION_BACKLOG.md)。
