# 平台适配层 ports/

`ports/` 是 **组件南向接口**（`ffl.driver_port`、OSAL 临界区、协议链路等）在  
**具体 MCU 系列**上的实现。组件 core 在 `components/`，**禁止**依赖厂商 HAL；  
把 HAL/时钟/总线翻成契约的代码都在这里。

```text
components/     通用逻辑
     ↓ 契约（ffl_transport_t / ffl_time_ops_t / 临界区钩子 …）
ports/<芯片>/   南向适配（本目录）
     ↓
应用 / 示例      业务 + 板级引脚选择
```

## 目录约定：**先芯片系列，再能力**

```text
ports/
├── stm32/f1/          # STM32F1（Cortex-M3）
│   ├── osal/          # 临界区
│   ├── driver_port/   # 时间 + I2C transport
│   ├── sc7a20/        # 传感器 bind 辅助
│   └── cherryusb/     # 第三方 USB 的 F1 适配
├── ch32/              # 沁恒 CH32
├── py32/              # Puya PY32
└── README.md
```

| 芯片 | 路径 | 已有能力 |
|------|------|----------|
| **STM32F1** | [`stm32/f1/`](stm32/f1/) | OSAL 临界区、I2C1+DWT 南向、SC7A20 bind、CherryUSB fsdev |
| **CH32** | [`ch32/`](ch32/) | adhoc 链路、SC7A20、SHT40；`legacy/` 非通用 port |
| **PY32** | [`py32/`](py32/) | OSAL 临界区/tick |

## 组件接入：两种情况

### A. 仓库已有该芯片 port（推荐）

应用 `xmake.lua`：

```lua
add_includedirs("path/to/ports/stm32/f1/sc7a20")
add_includedirs("path/to/ports/stm32/f1/driver_port")
add_files("path/to/ports/stm32/f1/driver_port/driver_port.c")
add_files("path/to/ports/stm32/f1/sc7a20/sc7a20_bind.c")
```

代码：

```c
ffl_stm32f1_sc7a20_bind_i2c1(&dev, /*remap=*/1, 0x18);
ffl_sc7a20_init(&dev, &cfg);
```

完整示例：[`examples/stm32-base-driver`](../examples/stm32-base-driver/docs/tasks/03-driver-with-port.md)

### B. 无 port（芯片未收录）→ **自行实现南向**

1. 读组件 `docs/` 与 `ffl/driver_port.h`。  
2. 实现 `xfer` / `time_ops` /（若需要）临界区。  
3. 在应用 `bsp/` 或新建 `ports/<你的芯片>/` 放置实现。  
4. 稳定后可贡献回 `ports/`。

**不要**为了接驱动去改 `components/` core。

## 接入已有 CH32/PY32 port

先读各 port README，确认 `board.h`、`drv_i2c.h`、全局句柄等前置条件，再在固件 target 里 `add_files` / `add_includedirs`。

## 自己编写 port 时

- 只实现组件实际需要的能力（transport、时间、GPIO、IRQ、临界区…）。  
- 厂厂 HAL 头 **只**出现在 `ports/` 或示例 `bsp/`，**不**进 `components/`。  
- host/mock 通过后，再在目标板记录硬件证据（见 `docs/maintainer/`）。
