#include "sht40_ch32_adapter.h"
#include "board.h"

#include <errno.h>

#define FFL_SHT40_CH32_DEFAULT_TIMEOUT_MS 50u

static int ffl_sht40_ch32_wait_idle(const ffl_sht40_ch32_i2c_t *bus)
{
    uint32_t start;
    uint32_t guard = 0u;
    uint32_t error;

    if (bus == 0) {
        return -EINVAL;
    }

    start = HAL_GetTick();
    while (1) {
        if (bsp_i2c_get_state(bus->i2c_num) == I2C_STATE_IDLE) {
            error = bsp_i2c_get_error(bus->i2c_num);
            return (error == I2C_OK) ? 0 : -EIO;
        }

        error = bsp_i2c_get_error(bus->i2c_num);
        if (error != I2C_OK) {
            return -EIO;
        }

        if ((HAL_GetTick() - start) >= bus->timeout_ms || guard++ >= 2000000u) {
            bsp_i2c_recover(bus->i2c_num);
            return -ETIMEDOUT;
        }
    }
}

static int ffl_sht40_ch32_xfer(void *ctx,
                               const ffl_endpoint_t *endpoint,
                               const ffl_xfer_msg_t *msgs,
                               uint8_t count,
                               ffl_xfer_done_fn done,
                               void *user)
{
    ffl_sht40_ch32_i2c_t *bus = (ffl_sht40_ch32_i2c_t *)ctx;
    const ffl_xfer_msg_t *message;
    uint8_t direction;
    int rc;

    if (bus == 0 || endpoint == 0 || msgs == 0 || count != 1u ||
        endpoint->kind != FFL_ENDPOINT_I2C_7BIT || !ffl_endpoint_is_valid(endpoint)) {
        return -EINVAL;
    }

    message = &msgs[0];
    direction = (uint8_t)(message->flags & FFL_XFER_MSG_DIRECTION_MASK);
    if (message->buf == 0 || message->len == 0u ||
        (direction != FFL_XFER_MSG_WRITE && direction != FFL_XFER_MSG_READ)) {
        return -EINVAL;
    }

    if (direction == FFL_XFER_MSG_READ) {
        rc = (bsp_i2c_read(bus->i2c_num,
                           endpoint->value.i2c.addr7,
                           message->buf,
                           message->len) == I2C_OK) ? 0 : -EIO;
    } else {
        rc = (bsp_i2c_write(bus->i2c_num,
                            endpoint->value.i2c.addr7,
                            message->buf,
                            message->len) == I2C_OK) ? 0 : -EIO;
    }
    if (rc == 0) {
        rc = ffl_sht40_ch32_wait_idle(bus);
    }

    if (rc == 0 && done != 0) {
        done(user, 0);
    }
    return rc;
}

static int ffl_sht40_ch32_cancel(void *ctx)
{
    ffl_sht40_ch32_i2c_t *bus = (ffl_sht40_ch32_i2c_t *)ctx;

    if (bus == 0) {
        return -EINVAL;
    }

    bsp_i2c_recover(bus->i2c_num);
    return 0;
}

static void ffl_sht40_ch32_delay_ms(void *ctx, uint32_t ms)
{
    (void)ctx;
    HAL_Delay(ms);
}

static const ffl_transport_ops_t g_ffl_sht40_ch32_transport_ops = {
    ffl_sht40_ch32_xfer,
    ffl_sht40_ch32_cancel,
};

static const ffl_time_ops_t g_ffl_sht40_ch32_time_ops = {
    .delay_ms = ffl_sht40_ch32_delay_ms,
    .delay_us = 0,
    .now_us = 0,
};

int ffl_sht40_ch32_transport_init(ffl_transport_t *transport,
                                   ffl_sht40_ch32_i2c_t *bus,
                                   i2c_num_t i2c_num,
                                   uint8_t addr7)
{
    if (transport == 0 || bus == 0 || i2c_num >= I2C_NUM_MAX || addr7 > 0x7Fu) {
        return -EINVAL;
    }

    bus->i2c_num = i2c_num;
    bus->timeout_ms = FFL_SHT40_CH32_DEFAULT_TIMEOUT_MS;
    transport->ops = &g_ffl_sht40_ch32_transport_ops;
    transport->ctx = bus;
    transport->endpoint = ffl_endpoint_i2c7(addr7);
    return 0;
}

void ffl_sht40_ch32_time_init(ffl_time_ops_t *time_ops)
{
    if (time_ops != 0) {
        *time_ops = g_ffl_sht40_ch32_time_ops;
    }
}
