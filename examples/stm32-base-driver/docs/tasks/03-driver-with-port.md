# Task 3：驱动组件 + MCU 南向适配（`ports/`）

驱动组件（如 `ffl.sc7a20`）**不**直接操作 I2C 寄存器，只通过  
**`ffl.driver_port` 南向接口**收总线与时间。  
**适配层（`ports/`）就是把具体 MCU 的 HAL 翻成这套契约。**

```text
ffl.sc7a20 (components/)  →  ffl_transport_t / ffl_time_ops_t
                ↑
     ports/stm32/f1/       ← 本系列已有南向
     或 应用内自写 port    ← 无 port 时
```

---

## 3.1 情况 A：仓库已有 port —— **有 port，直接接**

STM32F1 已提供：

| 路径 | 南向能力 |
|------|----------|
| `ports/stm32/f1/osal/` | `ffl.osal` / `sw_timer` 临界区 |
| `ports/stm32/f1/driver_port/` | `ffl_time_ops_t` + **通用** I2C `xfer`（实例由应用注入） |
| `ports/stm32/f1/sc7a20/` | `ffl_sc7a20_bind` 薄封装（仍要外部 transport） |

**port 不初始化 I2C、不选脚**。板级负责实例与引脚，再把句柄交给 port：

```c
#include "board_i2c.h"                         /* bsp：HAL_I2C_Init */
#include "ffl_port_stm32f1_driver_port.h"
#include "ffl_port_stm32f1_sc7a20.h"

static ffl_stm32f1_i2c_ctx_t i2c_ctx;
static ffl_transport_t i2c_tr;

board_i2c1_init_pb89();                        /* 板级：PB8/PB9 + I2C1 */
i2c_ctx.hi2c = board_i2c1_handle();
i2c_ctx.timeout_ms = 50;
ffl_stm32f1_i2c_transport_setup(&i2c_tr, &i2c_ctx, 0x18);
ffl_stm32f1_sc7a20_bind(&s_accel, &i2c_tr, 0x18);
ffl_sc7a20_init(&s_accel, &config);
```

同一 `ffl_stm32f1_i2c_ops()` 可挂 I2C1/I2C2 多份 transport。

同类已收录：`ports/ch32/sc7a20`、`ports/ch32/sht40`、`ports/py32/osal` 等，见  
[`ports/README.md`](../../../ports/README.md)。

---

## 3.2 情况 B：无 port —— **自行实现南向再接入**

若芯片不在 `ports/` 中（例如新 MCU），**不要改 sc7a20 core**，而是：

1. 实现 `ffl_transport_ops_t.xfer`（同步：`done==NULL`）  
2. 实现 `ffl_time_ops_t`（delay / now_us）  
3. 实现临界区（若用 OSAL / sw_timer）  
4. `ffl_sc7a20_bind(device, my_transport, my_time_ops, NULL)`  

本示例历史上曾把上述实现放在 `examples/stm32-base-driver/bsp/`  
（`sc7a20_i2c_transport.c`、`stm32_time_ops.c`、`osal_port_stm32.c`）；  
现已上收到 **`ports/stm32/f1/`**。`bsp/` 只保留板级时钟/LED/UART。

自写 port 最小模板（伪代码）：

```c
static int my_xfer(void *ctx, const ffl_endpoint_t *ep,
                   const ffl_xfer_msg_t *msgs, uint8_t n,
                   ffl_xfer_done_fn done, void *user) {
  if (done) return -ENOTSUP;          /* 先做同步 */
  /* msgs[0]=写寄存器, msgs[1]=读/写数据 → 操作你的 I2C */
  return 0; /* 或 -EIO / -ETIMEDOUT */
}
static const ffl_transport_ops_t ops = { .xfer = my_xfer, .cancel = my_cancel };
static ffl_transport_t tr = { .ops = &ops, .endpoint = { .kind = FFL_ENDPOINT_I2C_7BIT, ... } };
```

---

## 3.3 契约对照（SC7A20）

| core 发出的消息 | port 映射 |
|-----------------|-----------|
| `[WRITE reg][READ len]` | `HAL_I2C_Mem_Read` |
| `[WRITE reg][WRITE len]` | `HAL_I2C_Mem_Write` |

时间：复位/转换用 `delay_ms`；超时可用 `now_us`。

---

## 3.4 本板现象

```text
[sc7a20] who_am_i rc=0 val=0x11
[stats] sensor_ok=1 ok=... err=0
```

I2C1 重映射 PB8/PB9，由 `ffl_stm32f1_i2c1_transport_init(1, 100000)` 完成。

下一篇：[04-driver-no-hw.md](04-driver-no-hw.md)
