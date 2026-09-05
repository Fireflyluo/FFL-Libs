#include "sht30.h"

#include "ffl_atomic.h"

#include <errno.h>
#include <string.h>

static uint8_t sht30_async_state_load(sht30_dev_t *dev)
{
    return ffl_atomic_load_u8(&dev->async.state);
}

static bool sht30_async_transition(sht30_dev_t *dev, uint8_t expected, uint8_t desired)
{
    return ffl_atomic_compare_exchange_u8(&dev->async.state, expected, desired) != 0;
}

static void sht30_async_submit_begin(sht30_dev_t *dev)
{
    (void)ffl_atomic_fetch_add_u8(&dev->async.submit_depth, 1u);
}

static void sht30_async_submit_end(sht30_dev_t *dev)
{
    if (ffl_atomic_fetch_sub_u8(&dev->async.submit_depth, 1u) == 1u &&
        sht30_async_state_load(dev) == SHT30_ASYNC_STATE_IDLE) {
        sht30_core_unlock(dev);
    }
}

static bool sht30_async_consume_cancellation(sht30_dev_t *dev)
{
    return sht30_async_transition(dev,
                                  SHT30_ASYNC_STATE_CANCELLING,
                                  SHT30_ASYNC_STATE_CANCELLED);
}

static bool sht30_async_claim(sht30_dev_t *dev, uint8_t expected, uint8_t desired)
{
    uint8_t state;

    for (;;) {
        state = sht30_async_state_load(dev);
        if (state == SHT30_ASYNC_STATE_CANCELLING) {
            if (sht30_async_consume_cancellation(dev)) {
                return false;
            }
            continue;
        }
        if (state == SHT30_ASYNC_STATE_CANCELLED) {
            return false;
        }
        if (state != expected) {
            return false;
        }
        if (sht30_async_transition(dev, expected, desired)) {
            return true;
        }
    }
}

static bool sht30_async_finish(sht30_dev_t *dev, uint8_t expected_state)
{
    if (!sht30_async_claim(dev, expected_state, SHT30_ASYNC_STATE_IDLE)) {
        return false;
    }

    dev->async.op = SHT30_ASYNC_NONE;
    if (ffl_atomic_load_u8(&dev->async.submit_depth) == 0u) {
        sht30_core_unlock(dev);
    }
    return true;
}

static void sht30_async_read_done(void *user, int status)
{
    sht30_dev_t *dev = (sht30_dev_t *)user;
    sht30_async_ctx_t async;
    int rc;
    sht30_sample_t sample;

    if (dev == 0) {
        return;
    }

    if (!sht30_async_claim(dev,
                           SHT30_ASYNC_STATE_READ_PENDING,
                           SHT30_ASYNC_STATE_READ_PROCESSING) &&
        !sht30_async_claim(dev,
                           SHT30_ASYNC_STATE_READ_SUBMITTING,
                           SHT30_ASYNC_STATE_READ_PROCESSING)) {
        return;
    }

    async = dev->async;
    rc = sht30_core_map_bus_status(status);
    if (rc == 0 && async.op == SHT30_ASYNC_READ_SAMPLE) {
        rc = sht30_core_read_sample_parse(async.rx, &sample);
    }

    if (!sht30_async_finish(dev, SHT30_ASYNC_STATE_READ_PROCESSING)) {
        return;
    }

    if (async.op == SHT30_ASYNC_READ_SAMPLE && async.sample_cb != 0) {
        async.sample_cb(async.user, (rc == 0) ? &sample : 0, rc);
    } else if (async.done_cb != 0) {
        async.done_cb(async.user, rc);
    }
}

static void sht30_async_cmd_done(void *user, int status)
{
    sht30_dev_t *dev = (sht30_dev_t *)user;
    sht30_async_ctx_t async;
    sht30_comm_msg_t msg;
    int rc;
    bool finished = false;

    if (dev == 0 ||
        (!sht30_async_claim(dev,
                            SHT30_ASYNC_STATE_COMMAND_PENDING,
                            SHT30_ASYNC_STATE_COMMAND_PROCESSING) &&
         !sht30_async_claim(dev,
                            SHT30_ASYNC_STATE_COMMAND_SUBMITTING,
                            SHT30_ASYNC_STATE_COMMAND_PROCESSING))) {
        return;
    }

    async = dev->async;
    rc = sht30_core_map_bus_status(status);
    if (rc != 0) {
        if (sht30_async_finish(dev, SHT30_ASYNC_STATE_COMMAND_PROCESSING)) {
            if (async.op == SHT30_ASYNC_READ_SAMPLE && async.sample_cb != 0) {
                async.sample_cb(async.user, 0, rc);
            } else if (async.done_cb != 0) {
                async.done_cb(async.user, rc);
            }
        }
        return;
    }

    if (async.op == SHT30_ASYNC_SOFT_RESET) {
        if (dev->delay_ms != 0) {
            dev->delay_ms(dev->delay_ctx, 2u);
        }
        if (sht30_async_finish(dev, SHT30_ASYNC_STATE_COMMAND_PROCESSING) &&
            async.done_cb != 0) {
            async.done_cb(async.user, 0);
        }
        return;
    }

    if (async.op != SHT30_ASYNC_READ_SAMPLE) {
        if (sht30_async_finish(dev, SHT30_ASYNC_STATE_COMMAND_PROCESSING) &&
            async.done_cb != 0) {
            async.done_cb(async.user, -EINVAL);
        }
        return;
    }

    if (dev->delay_ms != 0) {
        dev->delay_ms(dev->delay_ctx, sht30_core_measure_delay_ms(async.cmd));
    }

    if (!sht30_async_claim(dev,
                           SHT30_ASYNC_STATE_COMMAND_PROCESSING,
                           SHT30_ASYNC_STATE_READ_SUBMITTING)) {
        return;
    }

    msg.buf = dev->async.rx;
    msg.len = 6u;
    msg.flags = (uint8_t)(SHT30_COMM_READ | SHT30_COMM_STOP);
    sht30_async_submit_begin(dev);
    rc = sht30_core_map_bus_status(dev->ops->xfer(dev->bus_ctx,
                                                  &msg,
                                                  1u,
                                                  sht30_async_read_done,
                                                  dev));
    if (rc != 0) {
        finished = sht30_async_finish(dev, SHT30_ASYNC_STATE_READ_SUBMITTING);
    }
    sht30_async_submit_end(dev);
    if (rc != 0 && finished) {
        if (async.sample_cb != 0) {
            async.sample_cb(async.user, 0, rc);
        }
        return;
    }

    (void)sht30_async_claim(dev,
                            SHT30_ASYNC_STATE_READ_SUBMITTING,
                            SHT30_ASYNC_STATE_READ_PENDING);
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
    ffl_atomic_store_u8(&dev->async.state, SHT30_ASYNC_STATE_COMMAND_SUBMITTING);

    msg.buf = dev->async.cmd;
    msg.len = 2u;
    msg.flags = (uint8_t)(SHT30_COMM_WRITE | SHT30_COMM_STOP);
    sht30_async_submit_begin(dev);
    rc = sht30_core_map_bus_status(dev->ops->xfer(dev->bus_ctx,
                                                  &msg,
                                                  1u,
                                                  sht30_async_cmd_done,
                                                  dev));
    if (rc != 0) {
        (void)sht30_async_finish(dev, SHT30_ASYNC_STATE_COMMAND_SUBMITTING);
    }
    sht30_async_submit_end(dev);
    if (rc != 0) {
        return rc;
    }
    (void)sht30_async_claim(dev,
                            SHT30_ASYNC_STATE_COMMAND_SUBMITTING,
                            SHT30_ASYNC_STATE_COMMAND_PENDING);
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
    ffl_atomic_store_u8(&dev->async.state, SHT30_ASYNC_STATE_COMMAND_SUBMITTING);

    msg.buf = dev->async.cmd;
    msg.len = 2u;
    msg.flags = (uint8_t)(SHT30_COMM_WRITE | SHT30_COMM_STOP);
    sht30_async_submit_begin(dev);
    rc = sht30_core_map_bus_status(dev->ops->xfer(dev->bus_ctx,
                                                  &msg,
                                                  1u,
                                                  sht30_async_cmd_done,
                                                  dev));
    if (rc != 0) {
        (void)sht30_async_finish(dev, SHT30_ASYNC_STATE_COMMAND_SUBMITTING);
    }
    sht30_async_submit_end(dev);
    if (rc != 0) {
        return rc;
    }
    (void)sht30_async_claim(dev,
                            SHT30_ASYNC_STATE_COMMAND_SUBMITTING,
                            SHT30_ASYNC_STATE_COMMAND_PENDING);
    return rc;
}

int sht30_cancel_async(sht30_dev_t *dev)
{
    uint8_t state;
    int rc;

    if (dev == 0) {
        return -EINVAL;
    }
    if (dev->ops == 0 || dev->ops->cancel == 0) {
        return -ENOTSUP;
    }

    for (;;) {
        state = sht30_async_state_load(dev);
        if (state == SHT30_ASYNC_STATE_IDLE) {
            return -ENOENT;
        }
        if (state == SHT30_ASYNC_STATE_CANCELLING ||
            state == SHT30_ASYNC_STATE_CANCELLED ||
            state == SHT30_ASYNC_STATE_COMMAND_SUBMITTING ||
            state == SHT30_ASYNC_STATE_COMMAND_PROCESSING ||
            state == SHT30_ASYNC_STATE_READ_SUBMITTING ||
            state == SHT30_ASYNC_STATE_READ_PROCESSING) {
            return -EBUSY;
        }
        if ((state == SHT30_ASYNC_STATE_COMMAND_PENDING ||
             state == SHT30_ASYNC_STATE_READ_PENDING) &&
            sht30_async_transition(dev, state, SHT30_ASYNC_STATE_CANCELLING)) {
            break;
        }
    }

    rc = sht30_core_map_bus_status(dev->ops->cancel(dev->bus_ctx));
    if (sht30_async_state_load(dev) == SHT30_ASYNC_STATE_CANCELLED) {
        dev->async.op = SHT30_ASYNC_NONE;
        ffl_atomic_store_u8(&dev->async.state, SHT30_ASYNC_STATE_IDLE);
        sht30_core_unlock(dev);
        return 0;
    }
    if (rc == 0) {
        dev->async.op = SHT30_ASYNC_NONE;
        ffl_atomic_store_u8(&dev->async.state, SHT30_ASYNC_STATE_IDLE);
        sht30_core_unlock(dev);
        return 0;
    }
    if (sht30_async_transition(dev, SHT30_ASYNC_STATE_CANCELLING, state)) {
        return rc;
    }
    if (sht30_async_state_load(dev) == SHT30_ASYNC_STATE_CANCELLED) {
        dev->async.op = SHT30_ASYNC_NONE;
        ffl_atomic_store_u8(&dev->async.state, SHT30_ASYNC_STATE_IDLE);
        sht30_core_unlock(dev);
        return 0;
    }
    return rc;
}
