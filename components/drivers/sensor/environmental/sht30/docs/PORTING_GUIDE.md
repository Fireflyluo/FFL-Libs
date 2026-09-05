# Porting Guide

SHT30 does not require an official MCU port. Adapt the project's I2C API to
`ffl_transport_ops_t`; this component does not include board or vendor HAL
headers. `ffl_sht30_bind()` requires `ffl_time_ops_t::delay_ms`, because reset
and measurement waits are mandatory for a correct SHT30 transaction.

## Minimal synchronous adapter

```c
#include "ffl/sht30.h"

static int board_i2c_xfer(void *ctx,
                          const ffl_endpoint_t *endpoint,
                          const ffl_xfer_msg_t *msgs,
                          uint8_t count,
                          ffl_xfer_done_fn done,
                          void *user)
{
    /* Translate endpoint->value.i2c.addr7 and msgs[] to the board I2C API. */
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

ffl_sht30_device_t sensor = {0};
ffl_sht30_config_t config;

ffl_sht30_config_init(&config);
ffl_sht30_bind(&sensor, &board_i2c, &board_time_ops, NULL);
ffl_sht30_init(&sensor, &config);
```

For a synchronous board I2C call, return its result directly and do not invoke
`done` when the caller does not request completion notification. When `done` is
provided, invoke `done(user, status)` exactly once after a successful transfer;
the callback may run before the adapter returns or after an asynchronous queue
operation. If submission fails synchronously, return the negative error and do
not invoke `done`. Implement `cancel` only when the board driver can abort a
pending asynchronous transfer.

There is currently no official SHT30 MCU adapter in this repository. Board I2C
initialization and hardware validation remain the consuming project's
responsibility.
