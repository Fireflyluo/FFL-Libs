#include "sc7a20_ch32_adapter.h"
#include "board.h"

#include <errno.h>

static int ch32_i2c_wait_idle(i2c_num_t i2c_num, uint32_t timeout_ms)
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

static int ch32_do_two_msg_xfer(sc7a20_ch32_bus_ctx_t *ctx, const sc7a20_comm_msg_t *msgs)
{
    const uint8_t reg = (uint8_t)(msgs[0].buf[0] & 0x7Fu);
    int rc;

    if ((msgs[0].flags & SC7A20_COMM_WRITE) == 0u || msgs[0].len != 1u) {
        return -EINVAL;
    }

    if ((msgs[1].flags & SC7A20_COMM_READ) != 0u) {
        rc = (bsp_i2c_read_register(ctx->i2c_num, ctx->dev_addr, reg, msgs[1].buf, msgs[1].len) == I2C_OK) ? 0 : -EIO;
        if (rc == 0) {
            rc = ch32_i2c_wait_idle(ctx->i2c_num, 50u);
        }
        return rc;
    }

    if ((msgs[1].flags & SC7A20_COMM_WRITE) != 0u) {
        rc = (bsp_i2c_write_register(ctx->i2c_num, ctx->dev_addr, reg, msgs[1].buf, msgs[1].len) == I2C_OK) ? 0 : -EIO;
        if (rc == 0) {
            rc = ch32_i2c_wait_idle(ctx->i2c_num, 50u);
        }
        return rc;
    }

    return -EINVAL;
}

static int ch32_i2c_xfer(void *bus_ctx,
                         const sc7a20_comm_msg_t *msgs,
                         uint8_t cnt,
                         sc7a20_bus_done_cb_t cb,
                         void *user)
{
    sc7a20_ch32_bus_ctx_t *ctx;
    int rc;

    if (bus_ctx == NULL || msgs == NULL || cnt == 0u) {
        return -EINVAL;
    }

    ctx = (sc7a20_ch32_bus_ctx_t *)bus_ctx;

    if (cnt == 2u) {
        rc = ch32_do_two_msg_xfer(ctx, msgs);
    } else {
        rc = -ENOTSUP;
    }

    if (cb != NULL) {
        cb(user, rc);
    }
    return rc;
}

static int ch32_i2c_cancel(void *bus_ctx)
{
    sc7a20_ch32_bus_ctx_t *ctx;

    if (bus_ctx == NULL) {
        return -EINVAL;
    }

    ctx = (sc7a20_ch32_bus_ctx_t *)bus_ctx;
    bsp_i2c_recover(ctx->i2c_num);
    return 0;
}

const sc7a20_bus_ops_t g_sc7a20_ch32_i2c_ops = {
    .xfer = ch32_i2c_xfer,
    .cancel = ch32_i2c_cancel,
};
