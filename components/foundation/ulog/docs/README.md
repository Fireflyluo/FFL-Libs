# ffl.ulog（foundation）

轻量异步日志库：格式化 → 环形缓冲 → `tx_try` 泵出到串口/USB 等后端。

来源：`D:\Desktop\ch32\Ad_Hoc_ALL_v3_worktree\v3\lib\ulog`（CH58x 工程）。

## 移植说明

| 原依赖 | 本仓库 |
|--------|--------|
| `CH58x_common.h` | 去掉 |
| `__risc_v_disable_irq` | Cortex-M 默认 PRIMASK；可 `ULOG_PORT_CUSTOM` + `ulog_port_lock/unlock` |
| `tmos_memcpy/memset` | 标准 `memcpy`/`memset` |

默认缓冲：`ULOG_BUFFER_SIZE=1024`，`ULOG_LINE_MAX=160`（比 CH 侧更省 RAM）。

## 用法

```c
#include "ulog.h"

static int uart_tx_try(void *ctx, const uint8_t *data, uint16_t len) {
  /* 尽力写入板级 UART 环/DMA，返回实际接收字节数 */
  return board_uart_write_try((const char *)data, len);
}

void app_log_init(void) {
  ulog_init_t cfg = {.tx_try = uart_tx_try, .tx_direct = NULL,
                     .tx_ctx = NULL, .poll_budget = 64};
  ulog_init(&cfg);
}

void app_loop(void) {
  ULOGI("hello %d", 42);
  ulog_poll(); /* 主循环泵出 */
}
```

- 任务上下文：`ULOGI/ULOGW/...`
- 中断：仅 `ulog_write_isr()`（无 `vsnprintf`）
- `tx_direct` 可选：ERROR 直发；失败仍回落环缓冲
