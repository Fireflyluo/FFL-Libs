# PORTING GUIDE

## 1. Bus contract

You only need to implement:

- `xfer(ctx, msgs, cnt, cb, user)`
- `cancel(ctx)` (optional)

`msgs` is an atomic transaction. Do not split it into unrelated calls from the driver side.

## 2. Message semantics

- `SC7A20_COMM_WRITE`: write message bytes to bus.
- `SC7A20_COMM_READ`: read bytes from bus.
- `SC7A20_COMM_STOP`: transaction end marker.

Typical register read:

1. write 1 byte register address
2. read N bytes

Typical register write:

1. write 1 byte register address
2. write N bytes payload

## 3. Sync vs async

- Sync API passes `cb = NULL` to `xfer`.
- Async API passes a completion callback.
- Adapter can implement both in one function.

## 4. CH32 integration example

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

## 5. Thread safety

The driver has an internal `in_use` lock to prevent re-entrant access.
If your adapter can complete from ISR context, ensure callback scheduling is safe for your system.
