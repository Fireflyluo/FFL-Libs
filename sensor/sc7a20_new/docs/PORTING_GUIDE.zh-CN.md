# 移植指南（中文）

> English version: [PORTING_GUIDE.md](./PORTING_GUIDE.md)

## 1. 总线契约

你只需要实现以下接口：

- `xfer(ctx, msgs, cnt, cb, user)`
- `cancel(ctx)`（可选）

`msgs` 表示一次原子事务，驱动层不会把它拆成互不相关的操作。

## 2. 消息语义

- `SC7A20_COMM_WRITE`: 写消息
- `SC7A20_COMM_READ`: 读消息
- `SC7A20_COMM_STOP`: 事务结束

典型寄存器读取事务：

1. 写 1 字节寄存器地址
2. 读 N 字节寄存器数据

典型寄存器写入事务：

1. 写 1 字节寄存器地址
2. 写 N 字节载荷

## 3. 同步与异步

- 同步 API：传 `cb = NULL` 给 `xfer`
- 异步 API：传完成回调给 `xfer`
- 一个适配器可同时支持同步与异步

## 4. CH32 接入示例

```c
#include "lib/sc7a20_new/inc/sc7a20.h"
#include "lib/sc7a20_new/adapters/sc7a20_ch32_adapter.h"

static sc7a20_dev_t g_sc7a20;
static sc7a20_ch32_bus_ctx_t g_sc7a20_bus = {
    .i2c_num = I2C_NUM_1,
    .dev_addr = SC7A20_I2C_ADDR_H,
};

static void sc7a20_setup(void)
{
    g_sc7a20.ops = &g_sc7a20_ch32_i2c_ops;
    g_sc7a20.bus_ctx = &g_sc7a20_bus;
    g_sc7a20.addr = g_sc7a20_bus.dev_addr;

    if (sc7a20_init(&g_sc7a20) != 0) {
        g_sc7a20_bus.dev_addr = SC7A20_I2C_ADDR_L;
        g_sc7a20.addr = g_sc7a20_bus.dev_addr;
        (void)sc7a20_init(&g_sc7a20);
    }
}
```

## 5. 并发与重入

驱动内部通过 `in_use` 锁避免重入。
如果底层回调运行在中断上下文，请确保你的调度策略与线程模型一致。
