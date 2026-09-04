#include "../inc/sc7a20.h"

#include <errno.h>
#include <string.h>

static void sc7a20_async_on_bus_done(void *user, int status)
{
    sc7a20_dev_t *dev;
    sc7a20_async_ctx_t async;
    int mapped;
    sc7a20_vec3i16_t xyz;

    dev = (sc7a20_dev_t *)user;
    if (dev == NULL) {
        return;
    }

    mapped = sc7a20_core_map_bus_status(status);
    async = dev->async;

    dev->async.op = SC7A20_ASYNC_OP_NONE;
    sc7a20_core_unlock(dev);

    if (mapped == 0 && async.op == SC7A20_ASYNC_OP_READ_XYZ && async.xyz_cb != NULL) {
        mapped = sc7a20_core_decode_xyz(dev, async.raw_xyz, &xyz);
        async.xyz_cb(async.user, (mapped == 0) ? &xyz : NULL, mapped);
        return;
    }

    if (async.op == SC7A20_ASYNC_OP_READ_XYZ && async.xyz_cb != NULL) {
        async.xyz_cb(async.user, NULL, mapped);
        return;
    }

    if (async.done_cb != NULL) {
        async.done_cb(async.user, mapped);
    }
}

int sc7a20_read_reg_async(sc7a20_dev_t *dev,
                          uint8_t reg,
                          uint8_t *data,
                          uint16_t len,
                          sc7a20_done_cb_t cb,
                          void *user)
{
    sc7a20_comm_msg_t msgs[2];
    int rc;

    if (dev == NULL || data == NULL || len == 0u || cb == NULL) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    rc = sc7a20_core_try_lock(dev);
    if (rc != 0) {
        return rc;
    }

    memset(&dev->async, 0, sizeof(dev->async));
    dev->async.op = SC7A20_ASYNC_OP_READ_REG;
    dev->async.reg_addr = (len > 1u) ? (uint8_t)(reg | 0x80u) : reg;
    dev->async.read_buf = data;
    dev->async.read_len = len;
    dev->async.done_cb = cb;
    dev->async.user = user;

    msgs[0].buf = &dev->async.reg_addr;
    msgs[0].len = 1u;
    msgs[0].flags = SC7A20_COMM_WRITE;

    msgs[1].buf = dev->async.read_buf;
    msgs[1].len = dev->async.read_len;
    msgs[1].flags = (uint8_t)(SC7A20_COMM_READ | SC7A20_COMM_STOP);

    rc = sc7a20_core_map_bus_status(dev->ops->xfer(dev->bus_ctx, msgs, 2u, sc7a20_async_on_bus_done, dev));
    if (rc != 0) {
        dev->async.op = SC7A20_ASYNC_OP_NONE;
        sc7a20_core_unlock(dev);
    }
    return rc;
}

int sc7a20_write_reg_async(sc7a20_dev_t *dev,
                           uint8_t reg,
                           const uint8_t *data,
                           uint16_t len,
                           sc7a20_done_cb_t cb,
                           void *user)
{
    sc7a20_comm_msg_t msgs[2];
    int rc;

    if (dev == NULL || data == NULL || len == 0u || cb == NULL) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    rc = sc7a20_core_try_lock(dev);
    if (rc != 0) {
        return rc;
    }

    memset(&dev->async, 0, sizeof(dev->async));
    dev->async.op = SC7A20_ASYNC_OP_WRITE_REG;
    dev->async.reg_addr = (len > 1u) ? (uint8_t)(reg | 0x80u) : reg;
    dev->async.write_buf = data;
    dev->async.write_len = len;
    dev->async.done_cb = cb;
    dev->async.user = user;

    msgs[0].buf = &dev->async.reg_addr;
    msgs[0].len = 1u;
    msgs[0].flags = SC7A20_COMM_WRITE;

    msgs[1].buf = (uint8_t *)dev->async.write_buf;
    msgs[1].len = dev->async.write_len;
    msgs[1].flags = (uint8_t)(SC7A20_COMM_WRITE | SC7A20_COMM_STOP);

    rc = sc7a20_core_map_bus_status(dev->ops->xfer(dev->bus_ctx, msgs, 2u, sc7a20_async_on_bus_done, dev));
    if (rc != 0) {
        dev->async.op = SC7A20_ASYNC_OP_NONE;
        sc7a20_core_unlock(dev);
    }
    return rc;
}

int sc7a20_read_xyz_raw_async(sc7a20_dev_t *dev, sc7a20_read_xyz_cb_t cb, void *user)
{
    sc7a20_comm_msg_t msgs[2];
    int rc;

    if (dev == NULL || cb == NULL) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    rc = sc7a20_core_try_lock(dev);
    if (rc != 0) {
        return rc;
    }

    memset(&dev->async, 0, sizeof(dev->async));
    dev->async.op = SC7A20_ASYNC_OP_READ_XYZ;
    dev->async.reg_addr = (uint8_t)(SC7A20_OUTX_L | 0x80u);
    dev->async.xyz_cb = cb;
    dev->async.user = user;

    msgs[0].buf = &dev->async.reg_addr;
    msgs[0].len = 1u;
    msgs[0].flags = SC7A20_COMM_WRITE;

    msgs[1].buf = dev->async.raw_xyz;
    msgs[1].len = 6u;
    msgs[1].flags = (uint8_t)(SC7A20_COMM_READ | SC7A20_COMM_STOP);

    rc = sc7a20_core_map_bus_status(dev->ops->xfer(dev->bus_ctx, msgs, 2u, sc7a20_async_on_bus_done, dev));
    if (rc != 0) {
        dev->async.op = SC7A20_ASYNC_OP_NONE;
        sc7a20_core_unlock(dev);
    }
    return rc;
}

int sc7a20_cancel_async(sc7a20_dev_t *dev)
{
    int rc;

    if (dev == NULL) {
        return -EINVAL;
    }
    rc = sc7a20_core_validate_dev(dev);
    if (rc != 0) {
        return rc;
    }
    if (dev->ops->cancel == NULL) {
        return -ENOTSUP;
    }

    rc = sc7a20_core_map_bus_status(dev->ops->cancel(dev->bus_ctx));
    if (rc == 0) {
        dev->async.op = SC7A20_ASYNC_OP_NONE;
        sc7a20_core_unlock(dev);
    }
    return rc;
}

