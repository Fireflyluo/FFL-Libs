# stm32-base-driver：FFL 仓库组件用法示例（LORA-minimal 合并固件）

本示例**以仓库用法为主线**，在 STM32F103 上用 **OSAL 多任务** 串起组件与外设。
组件源码来自 `components/`；南向在 `ports/stm32/f1/`；板级在 `bsp/`。

> **当前构建配置**：W25Q 使用 GPIO 软件 SPI；ST7789 使用 SPI1 + DMA；
> USB CDC 使用单包写后 ACK 协议将图像写入 W25Q；SC7A20 使用 I2C1。
> 引脚与硬件改线以 [`硬件映射表.md`](硬件映射表.md) 为准，构建通过不等同于板级硬件已验证。

## 任务一览（app/*_task.c）

| 文件 | 内容 |
|------|------|
| `heartbeat_task.c` | LED 心跳；蜂鸣 60s 两声 |
| `sc7a20_task.c` | 传感器 + ringbuffer + 统计 |
| `flash_task.c` | W25Q 软 SPI：JEDEC + 扇区探测 |
| `usb_img.c` | USB CDC 分包收图：CRC、写后 ACK、无大 RAM 环形缓存 |
| `lcd_task.c` | ST7789 刷屏；W25Q 擦写时跳过本拍避免抢软 SPI |
| `alarm_task.c` | UART 命令控制警报与图像切换（不承担图像上传） |
| `features_demo.c` | atomic / rb / sw_timer 自检 |

调试串口 COM4 @115200。

## USB 写图

图像只通过 USB CDC 写入，不使用调试 UART。主机按“发一包、等 ACK”传输：
每个 DATA 包最多 48B，带 CRC16；设备在该包已写入 W25Q 后才回复 ACK。
该背压方式只占一个 64B USB 接收缓冲，不需要 4KB 环形缓存。

```powershell
py -3 examples/stm32-base-driver/tools/usb_upload_img.py COM6
```

未指定 `COM6` 时，脚本会按 CDC 设备描述自动探测端口。详细帧格式见
[`app/usb_img.h`](app/usb_img.h)。

## 串口警报与切图

USART1（COM4，115200）使用固定 8 字节 ASCII 命令体，末尾接受 `LF`、`CR` 或 `CRLF`：

| 功能 | 报文 |
|------|------|
| 空袭 / 预先 / 解除 / 停止 | `cmd:ARDA` / `cmd:PREA` / `cmd:ACLR` / `cmd:STOP` + 行结束符 |
| 图像切换 | `cmd:IMG1` / `cmd:IMG2` / `cmd:IMG3` + 行结束符 |

调试 UART 不承担图像上传；USB CDC 的单包写后 ACK 下载协议保持不变。

板级接线（LORA-minimal）：LED=PA0，蜂鸣=PB6，UART1=COM4，I2C1=PB8/PB9，W25Q=软 SPI（PA15/PB3/4/5）。

---

## 学完你会分清

| 类型 | 是否要 MCU port（`ports/`） | 本示例中的例子 |
|------|-----------------------------|----------------|
| **纯软件组件** | 否 | `ffl.ringbuffer`、`ffl.sw_timer`、`ffl.atomic` |
| **运行时组件** | 要临界区 | `ffl.osal` + **`ports/stm32/f1/osal`** |
| **驱动 + 已有 port** | 仓库已做南向 | **`ports/stm32/f1/sc7a20` + `driver_port`** |
| **驱动 + 无 port** | 自实现南向 | 见 Task 3.2（历史实现曾在示例 `bsp/`） |
| **无硬件** | 降级 | `sensor_ok=0` 仍跑 ringbuffer/OSAL |

`ports/` 按**芯片系列**组织，总表：[`ports/README.md`](../../ports/README.md)

---

## 仓库分层（示例如何引用）

```text
应用 app/          业务：事件、采样、统计（不写寄存器）
    ↓
组件 components/   ffl.osal / sw_timer / ringbuffer / sc7a20
    ↓ 契约 ffl.driver_port
ports/stm32/f1/    MCU 南向：临界区、I2C、时间
    ↓
芯片 SDK/           STM32 HAL + CMSIS
```

接入方式（本工程 `xmake.lua`）：

```lua
-- 显式 add_files 组件源码（可裁剪、统一 -mcpu=cortex-m3）
add_files("../../components/runtime/osal/src/osal.c", ...)
```

生产工程也可用 `includes()` + `add_deps("ffl.sc7a20")`，见 `components/README.md`。

---

## 五个任务一览

| Task | 主题 | 文档 | 代码入口 |
|------|------|------|----------|
| 1 | 纯软件组件：ringbuffer + sw_timer | [tasks/01-foundation.md](docs/tasks/01-foundation.md) | `app_task.c` 采样写环、500ms 定时 |
| 2 | 运行时：osal 任务/事件/定时器 | [tasks/02-runtime-osal.md](docs/tasks/02-runtime-osal.md) | `app_task.c` LED/STATS 事件 |
| 3 | 驱动 + port：sc7a20 + driver_port | [tasks/03-driver-with-port.md](docs/tasks/03-driver-with-port.md) | `bsp/*transport*` + `ffl_sc7a20_*` |
| 4 | 无硬件：驱动失败仍跑通链路 | [tasks/04-driver-no-hw.md](docs/tasks/04-driver-no-hw.md) | `s_sensor_ok==0` 占位 0xFF |
| 5 | 组件功能点演示 | [tasks/05-features.md](docs/tasks/05-features.md) | `app/features_demo.c` |

建议顺序 1→2→3→4→5；也可只读 Task 3 若只关心驱动移植。

---

## 目录

```text
stm32-base-driver/
├── xmake.lua
├── readme.md                 ← 本文件（仓库用法入口）
├── docs/tasks/               ← 任务详解（中文）
├── app/
│   ├── main.c                ← 初始化顺序 + 主循环
│   ├── app_task.c            ← OSAL 任务：串起 1–4
│   └── features_demo.c       ← Task 5：atomic/rb/sw_timer 功能点
├── bsp/                      ← 仅 port，无业务
├── project/                  ← 链接脚本
└── sdk/                      ← 裁剪版 STM32F1 SDK
```

---

## 构建

```powershell
xmake f -P examples/stm32-base-driver -p cross --toolchain=arm-none-eabi -a arm -m release
xmake    -P examples/stm32-base-driver
```

## 串口观察（可选）

板级已实现 USART1 printf → COM4 @115200。上电后可见：

```text
[feat] atomic/rb/sw_timer self-check
[boot] stm32-lora component test, sensor_ok=1
[sc7a20] who_am_i ...
[stats] sensor_ok=1 ok=... err=... drained=...
```

无 SC7A20 时 `sensor_ok=0`，`err` 递增，`ok` 停住——用于验证 Task 4。

---

## 边界说明

| 本示例会 | 本示例不会 |
|----------|------------|
| 展示组件如何被应用调用 | 不把板级 GPIO 写进 `components/` |
| 用 `ffl.driver_port` 接 I2C | 不在组件里 `#include` STM32 HAL |
| 无传感器时仍演示调度链路 | 不把“编译通过”写成硬件已验证 |

硬件验证记录与 backlog 见 `docs/maintainer/`。
