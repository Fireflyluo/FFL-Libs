#include "ffl/sht30.h"
#include "sht30.h"

#include <errno.h>
#include <string.h>

static uint8_t ffl_sht30_map_flags(uint8_t flags)
{
    uint8_t mapped = 0u;

    if ((flags & SHT30_COMM_WRITE) != 0u) {
        mapped |= FFL_XFER_MSG_WRITE;
    }
    if ((flags & SHT30_COMM_READ) != 0u) {
        mapped |= FFL_XFER_MSG_READ;
    }
    if ((flags & SHT30_COMM_STOP) != 0u) {
        mapped |= FFL_XFER_MSG_STOP;
    }
    return mapped;
}

static int ffl_sht30_xfer_adapter(void *ctx,
                                  const sht30_comm_msg_t *msgs,
                                  uint8_t count,
                                  sht30_bus_done_cb_t done,
                                  void *user)
{
    sht30_dev_t *device = (sht30_dev_t *)ctx;
    ffl_xfer_msg_t message;
    ffl_endpoint_t endpoint;

    if (device == 0 || !ffl_transport_is_valid(device->transport) || msgs == 0 || count != 1u) {
        return -EINVAL;
    }

    endpoint = device->transport->endpoint;
    if (endpoint.kind != FFL_ENDPOINT_I2C_7BIT) {
        return -ENOTSUP;
    }
    endpoint.value.i2c.addr7 = device->addr;
    message.buf = msgs[0].buf;
    message.len = msgs[0].len;
    message.flags = ffl_sht30_map_flags(msgs[0].flags);

    return ffl_transport_xfer(device->transport, &endpoint, &message, 1u, done, user);
}

static int ffl_sht30_cancel_adapter(void *ctx)
{
    const sht30_dev_t *device = (const sht30_dev_t *)ctx;

    if (device == 0 || !ffl_transport_is_valid(device->transport)) {
        return -EINVAL;
    }
    return ffl_transport_cancel(device->transport);
}

static void ffl_sht30_delay_adapter(void *ctx, uint32_t ms)
{
    const sht30_dev_t *device = (const sht30_dev_t *)ctx;

    if (device != 0 && device->time_ops != 0 && device->time_ops->delay_ms != 0) {
        device->time_ops->delay_ms(device->time_ctx, ms);
    }
}

static const sht30_bus_ops_t g_ffl_sht30_transport_adapter = {
    .xfer = ffl_sht30_xfer_adapter,
    .cancel = ffl_sht30_cancel_adapter,
};

void ffl_sht30_config_init(ffl_sht30_config_t *config)
{
    if (config != 0) {
        config->i2c_addr7 = FFL_SHT30_DEFAULT_ADDR7;
    }
}

int ffl_sht30_bind(ffl_sht30_device_t *device,
                   const ffl_transport_t *transport,
                   const ffl_time_ops_t *time_ops,
                   void *time_ctx)
{
    int rc;

    if (device == 0 || !ffl_transport_is_valid(transport) ||
        transport->endpoint.kind != FFL_ENDPOINT_I2C_7BIT ||
        transport->endpoint.value.i2c.addr7 == 0u || time_ops == 0 ||
        time_ops->delay_ms == 0) {
        return -EINVAL;
    }

    rc = sht30_core_try_lock(device);
    if (rc != 0) {
        return rc;
    }
    if (device->initialized) {
        sht30_core_unlock(device);
        return -EBUSY;
    }

    device->ops = &g_ffl_sht30_transport_adapter;
    device->bus_ctx = device;
    device->transport = transport;
    device->time_ops = time_ops;
    device->time_ctx = time_ctx;
    device->delay_ms = (time_ops != 0 && time_ops->delay_ms != 0) ? ffl_sht30_delay_adapter : 0;
    device->delay_ctx = device;
    device->addr = transport->endpoint.value.i2c.addr7;
    memset(&device->async, 0, sizeof(device->async));
    device->initialized = false;
    sht30_core_unlock(device);
    return 0;
}

int ffl_sht30_set_i2c_addr(ffl_sht30_device_t *device, uint8_t addr7)
{
    int rc;

    if (device == 0 || addr7 == 0u || addr7 > 0x7Fu) {
        return -EINVAL;
    }

    rc = sht30_core_try_lock(device);
    if (rc != 0) {
        return rc;
    }
    device->addr = addr7;
    sht30_core_unlock(device);
    return 0;
}

int ffl_sht30_init(ffl_sht30_device_t *device, const ffl_sht30_config_t *config)
{
    int rc;

    if (device == 0) {
        return -EINVAL;
    }
    if (config != 0) {
        rc = ffl_sht30_set_i2c_addr(device, config->i2c_addr7);
        if (rc != 0) {
            return rc;
        }
    }
    return sht30_init(device);
}

int ffl_sht30_soft_reset(ffl_sht30_device_t *device)
{
    return sht30_soft_reset(device);
}

int ffl_sht30_read_status(ffl_sht30_device_t *device, uint16_t *status)
{
    return sht30_read_status(device, status);
}

int ffl_sht30_read_sample(ffl_sht30_device_t *device,
                          ffl_sht30_repeatability_t repeatability,
                          ffl_sht30_sample_t *out_sample)
{
    return sht30_read_sample(device, repeatability, out_sample);
}

int ffl_sht30_heater(ffl_sht30_device_t *device, ffl_sht30_heater_cmd_t command)
{
    return sht30_heater(device, command);
}

int ffl_sht30_soft_reset_async(ffl_sht30_device_t *device,
                               ffl_sht30_done_fn callback,
                               void *user)
{
    return sht30_soft_reset_async(device, callback, user);
}

int ffl_sht30_read_sample_async(ffl_sht30_device_t *device,
                                ffl_sht30_repeatability_t repeatability,
                                ffl_sht30_sample_fn callback,
                                void *user)
{
    return sht30_read_sample_async(device, repeatability, callback, user);
}

int ffl_sht30_cancel_async(ffl_sht30_device_t *device)
{
    return sht30_cancel_async(device);
}
