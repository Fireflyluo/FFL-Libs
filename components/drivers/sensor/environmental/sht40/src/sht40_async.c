#include "sht40.h"

#include "ffl_atomic.h"

#include <errno.h>
#include <string.h>

static uint8_t sht40_async_state_load(sht40_dev_t *dev)
{
    return ffl_atomic_load_u8(&dev->async.state);
}

static bool sht40_async_transition(sht40_dev_t *dev, uint8_t expected, uint8_t desired)
{
    return ffl_atomic_compare_exchange_u8(&dev->async.state, expected, desired) != 0;
}

static void sht40_async_submit_begin(sht40_dev_t *dev)
{
    (void)ffl_atomic_fetch_add_u8(&dev->async.submit_depth, 1u);
}

static void sht40_async_submit_end(sht40_dev_t *dev)
{
    if (ffl_atomic_fetch_sub_u8(&dev->async.submit_depth, 1u) == 1u &&
        sht40_async_state_load(dev) == SHT40_ASYNC_STATE_IDLE) {
        sht40_core_unlock(dev);
    }
}

static bool sht40_async_consume_cancellation(sht40_dev_t *dev)
{
    return sht40_async_transition(dev,
                                  SHT40_ASYNC_STATE_CANCELLING,
                                  SHT40_ASYNC_STATE_CANCELLED);
}

static bool sht40_async_claim(sht40_dev_t *dev, uint8_t expected, uint8_t desired)
{
    uint8_t state;

    for (;;) {
        state = sht40_async_state_load(dev);
        if (state == SHT40_ASYNC_STATE_CANCELLING) {
            if (sht40_async_consume_cancellation(dev)) {
                return false;
            }
            continue;
        }
        if (state == SHT40_ASYNC_STATE_CANCELLED) {
            return false;
        }
        if (state != expected) {
            return false;
        }
        if (sht40_async_transition(dev, expected, desired)) {
            return true;
        }
    }
}

static bool sht40_async_finish(sht40_dev_t *dev, uint8_t expected_state)
{
    if (!sht40_async_claim(dev, expected_state, SHT40_ASYNC_STATE_IDLE)) {
        return false;
    }

    dev->async.op = SHT40_ASYNC_NONE;
    if (ffl_atomic_load_u8(&dev->async.submit_depth) == 0u) {
        sht40_core_unlock(dev);
    }
    return true;
}

static void sht40_async_read_done(void *user, int status)
{
    sht40_dev_t *dev = (sht40_dev_t *)user;
    sht40_async_ctx_t async;
    int rc;
    sht40_sample_t sample;

    if (dev == 0) {
        return;
    }

    if (!sht40_async_claim(dev,
                           SHT40_ASYNC_STATE_READ_PENDING,
                           SHT40_ASYNC_STATE_READ_PROCESSING) &&
        !sht40_async_claim(dev,
                           SHT40_ASYNC_STATE_READ_SUBMITTING,
                           SHT40_ASYNC_STATE_READ_PROCESSING)) {
        return;
    }

    async = dev->async;
    rc = sht40_core_map_bus_status(status);
    if (rc == 0 && async.op == SHT40_ASYNC_READ_SAMPLE) {
        rc = sht40_core_read_sample_parse(async.rx, &sample);
    }

    if (!sht40_async_finish(dev, SHT40_ASYNC_STATE_READ_PROCESSING)) {
        return;
    }

    if (async.op == SHT40_ASYNC_READ_SAMPLE && async.sample_cb != 0) {
        async.sample_cb(async.user, (rc == 0) ? &sample : 0, rc);
    } else if (async.done_cb != 0) {
        async.done_cb(async.user, rc);
    }
}

static void sht40_async_cmd_done(void *user, int status)
{
    sht40_dev_t *dev = (sht40_dev_t *)user;
    sht40_async_ctx_t async;
    sht40_comm_msg_t msg;
    int rc;
    bool finished = false;

    if (dev == 0 ||
        (!sht40_async_claim(dev,
                            SHT40_ASYNC_STATE_COMMAND_PENDING,
                            SHT40_ASYNC_STATE_COMMAND_PROCESSING) &&
         !sht40_async_claim(dev,
                            SHT40_ASYNC_STATE_COMMAND_SUBMITTING,
                            SHT40_ASYNC_STATE_COMMAND_PROCESSING))) {
        return;
    }

    async = dev->async;
    rc = sht40_core_map_bus_status(status);
    if (rc != 0) {
        if (sht40_async_finish(dev, SHT40_ASYNC_STATE_COMMAND_PROCESSING)) {
            if (async.op == SHT40_ASYNC_READ_SAMPLE && async.sample_cb != 0) {
                async.sample_cb(async.user, 0, rc);
            } else if (async.done_cb != 0) {
                async.done_cb(async.user, rc);
            }
        }
        return;
    }

    if (async.op == SHT40_ASYNC_SOFT_RESET) {
        if (dev->delay_ms != 0) {
            dev->delay_ms(dev->delay_ctx, 2u);
        }
        if (sht40_async_finish(dev, SHT40_ASYNC_STATE_COMMAND_PROCESSING) &&
            async.done_cb != 0) {
            async.done_cb(async.user, 0);
        }
        return;
    }

    if (async.op != SHT40_ASYNC_READ_SAMPLE) {
        if (sht40_async_finish(dev, SHT40_ASYNC_STATE_COMMAND_PROCESSING) &&
            async.done_cb != 0) {
            async.done_cb(async.user, -EINVAL);
        }
        return;
    }

    if (dev->delay_ms != 0) {
        dev->delay_ms(dev->delay_ctx, sht40_core_measure_delay_ms(async.cmd));
    }

    if (!sht40_async_claim(dev,
                           SHT40_ASYNC_STATE_COMMAND_PROCESSING,
                           SHT40_ASYNC_STATE_READ_SUBMITTING)) {
        return;
    }

    msg.buf = dev->async.rx;
    msg.len = 6u;
    msg.flags = (uint8_t)(SHT40_COMM_READ | SHT40_COMM_STOP);
    sht40_async_submit_begin(dev);
    rc = sht40_core_map_bus_status(dev->ops->xfer(dev->bus_ctx,
                                                  &msg,
                                                  1u,
                                                  sht40_async_read_done,
                                                  dev));
    if (rc != 0) {
        finished = sht40_async_finish(dev, SHT40_ASYNC_STATE_READ_SUBMITTING);
    }
    sht40_async_submit_end(dev);
    if (rc != 0 && finished) {
        if (async.sample_cb != 0) {
            async.sample_cb(async.user, 0, rc);
        }
        return;
    }

    (void)sht40_async_claim(dev,
                            SHT40_ASYNC_STATE_READ_SUBMITTING,
                            SHT40_ASYNC_STATE_READ_PENDING);
}

int sht40_soft_reset_async(sht40_dev_t *dev, sht40_done_cb_t cb, void *user)
{
    sht40_comm_msg_t msg;
    int rc;

    if (dev == 0 || cb == 0) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    rc = sht40_core_try_lock(dev);
    if (rc != 0) {
        return rc;
    }

    memset(&dev->async, 0, sizeof(dev->async));
    dev->async.op = SHT40_ASYNC_SOFT_RESET;
    dev->async.cmd = 0x94u;
    dev->async.done_cb = cb;
    dev->async.user = user;
    ffl_atomic_store_u8(&dev->async.state, SHT40_ASYNC_STATE_COMMAND_SUBMITTING);

    msg.buf = &dev->async.cmd;
    msg.len = 1u;
    msg.flags = (uint8_t)(SHT40_COMM_WRITE | SHT40_COMM_STOP);
    sht40_async_submit_begin(dev);
    rc = sht40_core_map_bus_status(dev->ops->xfer(dev->bus_ctx,
                                                  &msg,
                                                  1u,
                                                  sht40_async_cmd_done,
                                                  dev));
    if (rc != 0) {
        (void)sht40_async_finish(dev, SHT40_ASYNC_STATE_COMMAND_SUBMITTING);
    }
    sht40_async_submit_end(dev);
    if (rc != 0) {
        return rc;
    }
    (void)sht40_async_claim(dev,
                            SHT40_ASYNC_STATE_COMMAND_SUBMITTING,
                            SHT40_ASYNC_STATE_COMMAND_PENDING);
    return rc;
}

int sht40_read_sample_async(sht40_dev_t *dev,
                            sht40_precision_t precision,
                            sht40_sample_cb_t cb,
                            void *user)
{
    sht40_comm_msg_t msg;
    int rc;

    if (dev == 0 || cb == 0) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    rc = sht40_core_try_lock(dev);
    if (rc != 0) {
        return rc;
    }

    memset(&dev->async, 0, sizeof(dev->async));
    dev->async.op = SHT40_ASYNC_READ_SAMPLE;
    dev->async.cmd = sht40_core_precision_cmd(precision);
    dev->async.sample_cb = cb;
    dev->async.user = user;
    ffl_atomic_store_u8(&dev->async.state, SHT40_ASYNC_STATE_COMMAND_SUBMITTING);

    msg.buf = &dev->async.cmd;
    msg.len = 1u;
    msg.flags = (uint8_t)(SHT40_COMM_WRITE | SHT40_COMM_STOP);
    sht40_async_submit_begin(dev);
    rc = sht40_core_map_bus_status(dev->ops->xfer(dev->bus_ctx,
                                                  &msg,
                                                  1u,
                                                  sht40_async_cmd_done,
                                                  dev));
    if (rc != 0) {
        (void)sht40_async_finish(dev, SHT40_ASYNC_STATE_COMMAND_SUBMITTING);
    }
    sht40_async_submit_end(dev);
    if (rc != 0) {
        return rc;
    }
    (void)sht40_async_claim(dev,
                            SHT40_ASYNC_STATE_COMMAND_SUBMITTING,
                            SHT40_ASYNC_STATE_COMMAND_PENDING);
    return rc;
}

int sht40_cancel_async(sht40_dev_t *dev)
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
        state = sht40_async_state_load(dev);
        if (state == SHT40_ASYNC_STATE_IDLE) {
            return -ENOENT;
        }
        if (state == SHT40_ASYNC_STATE_CANCELLING ||
            state == SHT40_ASYNC_STATE_CANCELLED ||
            state == SHT40_ASYNC_STATE_COMMAND_SUBMITTING ||
            state == SHT40_ASYNC_STATE_COMMAND_PROCESSING ||
            state == SHT40_ASYNC_STATE_READ_SUBMITTING ||
            state == SHT40_ASYNC_STATE_READ_PROCESSING) {
            return -EBUSY;
        }
        if ((state == SHT40_ASYNC_STATE_COMMAND_PENDING ||
             state == SHT40_ASYNC_STATE_READ_PENDING) &&
            sht40_async_transition(dev, state, SHT40_ASYNC_STATE_CANCELLING)) {
            break;
        }
    }

    rc = sht40_core_map_bus_status(dev->ops->cancel(dev->bus_ctx));
    if (sht40_async_state_load(dev) == SHT40_ASYNC_STATE_CANCELLED) {
        dev->async.op = SHT40_ASYNC_NONE;
        ffl_atomic_store_u8(&dev->async.state, SHT40_ASYNC_STATE_IDLE);
        sht40_core_unlock(dev);
        return 0;
    }
    if (rc == 0) {
        dev->async.op = SHT40_ASYNC_NONE;
        ffl_atomic_store_u8(&dev->async.state, SHT40_ASYNC_STATE_IDLE);
        sht40_core_unlock(dev);
        return 0;
    }
    if (sht40_async_transition(dev, SHT40_ASYNC_STATE_CANCELLING, state)) {
        return rc;
    }
    if (sht40_async_state_load(dev) == SHT40_ASYNC_STATE_CANCELLED) {
        dev->async.op = SHT40_ASYNC_NONE;
        ffl_atomic_store_u8(&dev->async.state, SHT40_ASYNC_STATE_IDLE);
        sht40_core_unlock(dev);
        return 0;
    }
    return rc;
}
