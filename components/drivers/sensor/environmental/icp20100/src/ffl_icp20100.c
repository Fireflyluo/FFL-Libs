#include "ffl/icp20100.h"

#include "ffl_atomic.h"

#include <errno.h>
#include <string.h>

static uint8_t ffl_icp20100_map_flags(uint8_t flags)
{
    uint8_t mapped = 0u;

    if ((flags & ICP20100_COMM_WRITE) != 0u) {
        mapped |= FFL_XFER_MSG_WRITE;
    }
    if ((flags & ICP20100_COMM_READ) != 0u) {
        mapped |= FFL_XFER_MSG_READ;
    }
    if ((flags & ICP20100_COMM_STOP) != 0u) {
        mapped |= FFL_XFER_MSG_STOP;
    }
    return mapped;
}

static int ffl_icp20100_try_lock(ffl_icp20100_device_t *device)
{
    if (device == 0) {
        return -EINVAL;
    }
    return ffl_atomic_try_lock_u8(&device->in_use) ? 0 : -EBUSY;
}

static void ffl_icp20100_unlock(ffl_icp20100_device_t *device)
{
    if (device != 0) {
        ffl_atomic_unlock_u8(&device->in_use);
    }
}

static int ffl_icp20100_xfer_adapter(void *ctx,
                                     const icp20100_comm_msg_t *msgs,
                                     uint8_t count,
                                     void *done_cb,
                                     void *user)
{
    icp20100_dev_t *device = (icp20100_dev_t *)ctx;
    ffl_xfer_msg_t transport_msgs[2];
    ffl_endpoint_t endpoint;
    uint8_t index;

    if (device == 0 || !ffl_transport_is_valid(device->transport) || msgs == 0 ||
        count == 0u || count > (uint8_t)(sizeof(transport_msgs) / sizeof(transport_msgs[0]))) {
        return -EINVAL;
    }
    if (done_cb != 0) {
        return -ENOTSUP;
    }

    endpoint = device->transport->endpoint;
    if (endpoint.kind != FFL_ENDPOINT_I2C_7BIT) {
        return -ENOTSUP;
    }
    endpoint.value.i2c.addr7 = device->addr;

    for (index = 0u; index < count; ++index) {
        transport_msgs[index].buf = msgs[index].buf;
        transport_msgs[index].len = msgs[index].len;
        transport_msgs[index].flags = ffl_icp20100_map_flags(msgs[index].flags);
    }
    return ffl_transport_xfer(device->transport, &endpoint, transport_msgs, count, 0, user);
}

static void ffl_icp20100_delay_us_adapter(void *ctx, uint32_t us)
{
    const icp20100_dev_t *device = (const icp20100_dev_t *)ctx;

    if (device != 0 && device->time_ops != 0 && device->time_ops->delay_us != 0) {
        device->time_ops->delay_us(device->time_ctx, us);
    }
}

static const icp20100_bus_ops_t g_ffl_icp20100_transport_adapter = {
    .xfer = ffl_icp20100_xfer_adapter,
};

void ffl_icp20100_config_init(ffl_icp20100_config_t *config)
{
    if (config != 0) {
        *config = g_icp20100_default_cfg;
    }
}

int ffl_icp20100_bind(ffl_icp20100_device_t *device,
                      const ffl_transport_t *transport,
                      const ffl_time_ops_t *time_ops,
                      void *time_ctx)
{
    int rc;

    if (device == 0 || !ffl_transport_is_valid(transport) ||
        transport->endpoint.kind != FFL_ENDPOINT_I2C_7BIT ||
        transport->endpoint.value.i2c.addr7 == 0u || time_ops == 0 ||
        time_ops->delay_us == 0) {
        return -EINVAL;
    }
    rc = ffl_icp20100_try_lock(device);
    if (rc != 0) {
        return rc;
    }
    if (device->initialized) {
        ffl_icp20100_unlock(device);
        return -EBUSY;
    }

    device->ops = &g_ffl_icp20100_transport_adapter;
    device->bus_ctx = device;
    device->delay_us = ffl_icp20100_delay_us_adapter;
    device->delay_ctx = device;
    device->transport = transport;
    device->time_ops = time_ops;
    device->time_ctx = time_ctx;
    device->addr = transport->endpoint.value.i2c.addr7;
    device->chip_id = 0u;
    device->version = 0u;
    memset(&device->cfg, 0, sizeof(device->cfg));
    ffl_icp20100_unlock(device);
    return 0;
}

int ffl_icp20100_set_i2c_addr(ffl_icp20100_device_t *device, uint8_t addr7)
{
    int rc;

    if (device == 0 || addr7 == 0u || addr7 > 0x7Fu) {
        return -EINVAL;
    }
    rc = ffl_icp20100_try_lock(device);
    if (rc != 0) {
        return rc;
    }
    if (device->initialized) {
        ffl_icp20100_unlock(device);
        return -EBUSY;
    }

    device->addr = addr7;
    ffl_icp20100_unlock(device);
    return 0;
}

int ffl_icp20100_init(ffl_icp20100_device_t *device,
                      const ffl_icp20100_config_t *config)
{
    ffl_icp20100_config_t local_config;
    int rc;

    if (device == 0) {
        return -EINVAL;
    }

    rc = ffl_icp20100_try_lock(device);
    if (rc != 0) {
        return rc;
    }
    if (config != 0) {
        local_config = *config;
    } else {
        local_config = g_icp20100_default_cfg;
        local_config.addr = device->addr;
    }
    rc = icp20100_init(device, &local_config);
    ffl_icp20100_unlock(device);
    return rc;
}

int ffl_icp20100_probe(ffl_icp20100_device_t *device,
                       uint8_t *chip_id,
                       uint8_t *version)
{
    int rc = ffl_icp20100_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = icp20100_probe(device, chip_id, version);
    ffl_icp20100_unlock(device);
    return rc;
}

int ffl_icp20100_stop_measurement(ffl_icp20100_device_t *device)
{
    int rc = ffl_icp20100_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = icp20100_stop_measurement(device);
    ffl_icp20100_unlock(device);
    return rc;
}

int ffl_icp20100_configure(ffl_icp20100_device_t *device,
                            const ffl_icp20100_config_t *config)
{
    int rc = ffl_icp20100_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = icp20100_set_config(device, config);
    ffl_icp20100_unlock(device);
    return rc;
}

int ffl_icp20100_read_raw(ffl_icp20100_device_t *device,
                           ffl_icp20100_raw_sample_t *raw_sample)
{
    int rc = ffl_icp20100_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = icp20100_read_raw(device, raw_sample);
    ffl_icp20100_unlock(device);
    return rc;
}

int ffl_icp20100_read_sample(ffl_icp20100_device_t *device,
                              ffl_icp20100_sample_t *sample)
{
    int rc = ffl_icp20100_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = icp20100_read_sample(device, sample);
    ffl_icp20100_unlock(device);
    return rc;
}
