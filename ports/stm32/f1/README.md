# STM32F1 南向适配（ports/stm32/f1）

本目录实现 **组件契约 → STM32F1 HAL** 的翻译，**不**选择板级引脚、**不**硬编码某一个 I2C 实例。

```text
components/     通用逻辑（无 HAL）
     ↓ 契约
ports/stm32/f1/ 临界区、I2C xfer、时间、bind 辅助
     ↓ 应用注入 I2C_HandleTypeDef* / 时钟 / GPIO
app + bsp       板级初始化
```

## 设计原则

| ports 做 | ports 不做 |
|----------|------------|
| `ffl_transport_ops_t.xfer` 映射到 `HAL_I2C_Mem_*` | `HAL_I2C_Init`、选 I2C1/I2C2 |
| `ffl_time_ops_t`（DWT） | 选择用哪个定时器当 tick |
| OSAL 临界区（PRIMASK） | 业务事件编排 |
| 可选 bind 薄封装 | 写死板级引脚 |

应用准备就绪后：

```c
ffl_stm32f1_i2c_ctx_t ctx = { .hi2c = board_i2c1_handle(), .timeout_ms = 50 };
ffl_transport_t tr;
ffl_stm32f1_i2c_transport_setup(&tr, &ctx, /*addr7=*/0x18);
ffl_stm32f1_sc7a20_bind(&dev, &tr, 0x18);
```

同一套 ops 可挂多个 transport（I2C1、I2C2 各一份 ctx）。

## 子模块

| 路径 | 能力 |
|------|------|
| [`osal/`](osal/) | `ffl.osal` / `sw_timer` 临界区钩子 |
| [`driver_port/`](driver_port/) | DWT `ffl_time_ops_t` + 通用 I2C transport |
| [`sc7a20/`](sc7a20/) | `ffl_sc7a20_bind` 薄封装（仍要求外部 transport） |
| [`cherryusb/stm32-lora/`](cherryusb/stm32-lora/) | 第三方 USB fsdev 板级时钟/NVIC |

## 有 port / 无 port

- **有 port：** 见 [`examples/stm32-base-driver/docs/tasks/03-driver-with-port.md`](../../../examples/stm32-base-driver/docs/tasks/03-driver-with-port.md)
- **无 port：** 自行实现 `xfer`/`time_ops`，勿改 `components/`，详见同文档 3.2

## 依赖

- `stm32f1xx_hal.h`（`USE_HAL_DRIVER`）
- 应用已配置系统时钟；I2C 由应用 `HAL_I2C_Init` 后注入
