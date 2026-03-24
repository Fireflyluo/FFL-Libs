#include "sht40_ch32_adapter.h"
#include "board.h"

#include <errno.h>

static int sht40_i2c_wait_idle(i2c_num_t i2c_num, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint32_t guard = 0u;
    uint32_t err;

    while (1) {
        if (bsp_i2c_get_state(i2c_num) == I2C_STATE_IDLE) {
            err = bsp_i2c_get_error(i2c_num);
            return (err == I2C_OK) ? 0 : -EIO;
        }

        err = bsp_i2c_get_error(i2c_num);
        if (err != I2C_OK) {
            return -EIO;
        }

        if ((HAL_GetTick() - start) >= timeout_ms) {
            bsp_i2c_recover(i2c_num);
            return -ETIMEDOUT;
        }

        guard++;
        if (guard > 2000000u) {
            bsp_i2c_recover(i2c_num);
            return -ETIMEDOUT;
        }
    }
}

static int sht40_ch32_xfer(void *ctx,
                           const sht40_comm_msg_t *msgs,
                           uint8_t cnt,
                           sht40_bus_done_cb_t cb,
                           void *user)
{
    sht40_ch32_bus_ctx_t *bus;
    int rc;

    if (ctx == 0 || msgs == 0 || cnt == 0u) {
        return -EINVAL;
    }

    bus = (sht40_ch32_bus_ctx_t *)ctx;

    if (cnt == 1u) {
        if ((msgs[0].flags & SHT40_COMM_READ) != 0u) {
            rc = (bsp_i2c_read(bus->i2c_num, bus->dev_addr, msgs[0].buf, msgs[0].len) == I2C_OK) ? 0 : -EIO;
            if (rc == 0) {
                rc = sht40_i2c_wait_idle(bus->i2c_num, 50u);
            }
        } else if ((msgs[0].flags & SHT40_COMM_WRITE) != 0u) {
            rc = (bsp_i2c_write(bus->i2c_num, bus->dev_addr, msgs[0].buf, msgs[0].len) == I2C_OK) ? 0 : -EIO;
            if (rc == 0) {
                rc = sht40_i2c_wait_idle(bus->i2c_num, 50u);
            }
        } else {
            rc = -EINVAL;
        }
    } else {
        rc = -ENOTSUP;
    }

    if (cb != 0) {
        cb(user, rc);
    }
    return rc;
}

static int sht40_ch32_cancel(void *ctx)
{
    sht40_ch32_bus_ctx_t *bus;

    if (ctx == 0) {
        return -EINVAL;
    }

    bus = (sht40_ch32_bus_ctx_t *)ctx;
    bsp_i2c_recover(bus->i2c_num);
    return 0;
}

const sht40_bus_ops_t g_sht40_ch32_i2c_ops = {
    .xfer = sht40_ch32_xfer,
    .cancel = sht40_ch32_cancel,
};
