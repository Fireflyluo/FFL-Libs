#include "sht30.h"

#include <errno.h>
#include <string.h>

static void sht30_async_read_done(void *user, int status)
{
    sht30_dev_t *dev = (sht30_dev_t *)user;
    sht30_async_ctx_t async;
    int rc;
    sht30_sample_t sample;

    if (dev == 0) {
        return;
    }

    async = dev->async;
    dev->async.op = SHT30_ASYNC_NONE;
    sht30_core_unlock(dev);

    rc = sht30_core_map_bus_status(status);
    if (rc == 0 && async.op == SHT30_ASYNC_READ_SAMPLE && async.sample_cb != 0) {
        rc = sht30_core_read_sample_parse(async.rx, &sample);
        async.sample_cb(async.user, (rc == 0) ? &sample : 0, rc);
        return;
    }

    if (async.op == SHT30_ASYNC_READ_SAMPLE && async.sample_cb != 0) {
        async.sample_cb(async.user, 0, rc);
        return;
    }

    if (async.done_cb != 0) {
        async.done_cb(async.user, rc);
    }
}

static void sht30_async_cmd_done(void *user, int status)
{
    sht30_dev_t *dev = (sht30_dev_t *)user;
    sht30_comm_msg_t msg;
    int rc;

    if (dev == 0) {
        return;
    }

    rc = sht30_core_map_bus_status(status);
    if (rc != 0) {
        sht30_async_read_done(user, rc);
        return;
    }

    if (dev->async.op == SHT30_ASYNC_SOFT_RESET) {
        if (dev->delay_ms != 0) {
            dev->delay_ms(dev->delay_ctx, 2u);
        }
        sht30_async_read_done(user, 0);
        return;
    }

    if (dev->delay_ms != 0) {
        dev->delay_ms(dev->delay_ctx, sht30_core_measure_delay_ms(dev->async.cmd));
    }

    msg.buf = dev->async.rx;
    msg.len = 6u;
    msg.flags = (uint8_t)(SHT30_COMM_READ | SHT30_COMM_STOP);

    rc = sht30_core_map_bus_status(dev->ops->xfer(dev->bus_ctx, &msg, 1u, sht30_async_read_done, dev));
    if (rc != 0) {
        sht30_async_read_done(user, rc);
    }
}

int sht30_soft_reset_async(sht30_dev_t *dev, sht30_done_cb_t cb, void *user)
{
    sht30_comm_msg_t msg;
    int rc;

    if (dev == 0 || cb == 0) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    rc = sht30_core_try_lock(dev);
    if (rc != 0) {
        return rc;
    }

    memset(&dev->async, 0, sizeof(dev->async));
    dev->async.op = SHT30_ASYNC_SOFT_RESET;
    dev->async.cmd[0] = 0x30u;
    dev->async.cmd[1] = 0xA2u;
    dev->async.done_cb = cb;
    dev->async.user = user;

    msg.buf = dev->async.cmd;
    msg.len = 2u;
    msg.flags = (uint8_t)(SHT30_COMM_WRITE | SHT30_COMM_STOP);

    rc = sht30_core_map_bus_status(dev->ops->xfer(dev->bus_ctx, &msg, 1u, sht30_async_cmd_done, dev));
    if (rc != 0) {
        dev->async.op = SHT30_ASYNC_NONE;
        sht30_core_unlock(dev);
    }
    return rc;
}

int sht30_read_sample_async(sht30_dev_t *dev,
                            sht30_repeatability_t repeatability,
                            sht30_sample_cb_t cb,
                            void *user)
{
    sht30_comm_msg_t msg;
    int rc;

    if (dev == 0 || cb == 0) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    rc = sht30_core_try_lock(dev);
    if (rc != 0) {
        return rc;
    }

    memset(&dev->async, 0, sizeof(dev->async));
    dev->async.op = SHT30_ASYNC_READ_SAMPLE;
    sht30_core_precision_cmd(dev->async.cmd, repeatability);
    dev->async.sample_cb = cb;
    dev->async.user = user;

    msg.buf = dev->async.cmd;
    msg.len = 2u;
    msg.flags = (uint8_t)(SHT30_COMM_WRITE | SHT30_COMM_STOP);

    rc = sht30_core_map_bus_status(dev->ops->xfer(dev->bus_ctx, &msg, 1u, sht30_async_cmd_done, dev));
    if (rc != 0) {
        dev->async.op = SHT30_ASYNC_NONE;
        sht30_core_unlock(dev);
    }
    return rc;
}

int sht30_cancel_async(sht30_dev_t *dev)
{
    int rc;

    if (dev == 0) {
        return -EINVAL;
    }
    if (dev->ops == 0 || dev->ops->cancel == 0) {
        return -ENOTSUP;
    }

    rc = sht30_core_map_bus_status(dev->ops->cancel(dev->bus_ctx));
    if (rc == 0) {
        dev->async.op = SHT30_ASYNC_NONE;
        sht30_core_unlock(dev);
    }
    return rc;
}