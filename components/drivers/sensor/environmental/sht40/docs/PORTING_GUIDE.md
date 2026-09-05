# PORTING GUIDE

The SHT40 core has no required MCU port. A consuming project may adapt its
own I2C API to `ffl_transport_ops_t`; no board header or vendor HAL header is
included by this component. `ffl_sht40_bind()` requires `ffl_time_ops_t::delay_ms`,
because reset and measurement waits are mandatory for a correct transaction.

## Minimal synchronous adapter

```c
#include "ffl/sht40.h"

static int board_i2c_xfer(void *ctx,
                          const ffl_endpoint_t *endpoint,
                          const ffl_xfer_msg_t *msgs,
                          uint8_t count,
                          ffl_xfer_done_fn done,
                          void *user)
{
    /* Translate endpoint->value.i2c.addr7 and msgs[] to the board I2C API. */
    /* Call done(user, status) only when completing an asynchronous request. */
    (void)ctx;
    (void)endpoint;
    (void)msgs;
    (void)count;
    (void)done;
    (void)user;
    return 0;
}

static void board_delay_ms(void *ctx, uint32_t ms)
{
    (void)ctx;
    /* Delay for ms milliseconds. */
}

static const ffl_transport_ops_t board_i2c_ops = {
    .xfer = board_i2c_xfer,
    .cancel = NULL,
};

static const ffl_time_ops_t board_time_ops = {
    .delay_ms = board_delay_ms,
    .now_us = NULL,
};

static ffl_transport_t board_i2c = {
    .ops = &board_i2c_ops,
    .ctx = NULL,
    .endpoint = { .kind = FFL_ENDPOINT_I2C_7BIT, .value.i2c = { .addr7 = 0x44 } },
};

ffl_sht40_device_t sensor = {0};
ffl_sht40_config_t config;

ffl_sht40_config_init(&config);
ffl_sht40_bind(&sensor, &board_i2c, &board_time_ops, NULL);
ffl_sht40_init(&sensor, &config);
```

For a synchronous board I2C call, return its result directly and do not invoke
`done` when the caller does not request completion notification. When `done` is
provided, invoke `done(user, status)` exactly once after a successful transfer;
the callback may run before the adapter returns or after an asynchronous queue
operation. A delayed callback must run in task or thread context, not directly
from an ISR. If submission fails synchronously, return the negative error and do
not invoke `done`. Implement
`cancel` only when the board driver can abort a pending asynchronous transfer.

## Official CH32 adapter

`ports/ch32/sht40/` provides the reusable CH32 adapter for the repository's
`drv_i2c` API. It does not change the core target and is not included in host
tests because it requires CH32 vendor headers and board symbols:

```c
#include "sht40_ch32_adapter.h"

ffl_sht40_ch32_i2c_t bus;
ffl_transport_t transport;
ffl_time_ops_t time_ops;
ffl_sht40_device_t sensor = {0};
ffl_sht40_config_t config;

ffl_sht40_ch32_transport_init(&transport, &bus, I2C_NUM_1, 0x44u);
ffl_sht40_ch32_time_init(&time_ops);
ffl_sht40_config_init(&config);
config.i2c_addr7 = 0x44u;
ffl_sht40_bind(&sensor, &transport, &time_ops, NULL);
ffl_sht40_init(&sensor, &config);
```

The adapter uses the endpoint address for every transaction, waits for the
CH32 I2C state to become idle, and requests bus recovery on timeout or cancel.
The adapter still requires board-level I2C initialization before use.
