#include "ffl/qmi8658a.h"

#include "ffl_atomic.h"

#include <errno.h>

static int ffl_qmi8658a_try_lock(ffl_qmi8658a_device_t *device)
{
    if (device == 0) {
        return -EINVAL;
    }
    return ffl_atomic_try_lock_u8(&device->in_use) ? 0 : -EBUSY;
}

static void ffl_qmi8658a_unlock(ffl_qmi8658a_device_t *device)
{
    if (device != 0) {
        ffl_atomic_unlock_u8(&device->in_use);
    }
}

static uint8_t ffl_qmi8658a_map_flags(uint8_t flags, uint8_t index)
{
    uint8_t mapped = 0u;

    if ((flags & IMU_BUS_MSG_WRITE) != 0u) {
        mapped |= FFL_XFER_MSG_WRITE;
    }
    if ((flags & IMU_BUS_MSG_READ) != 0u) {
        mapped |= FFL_XFER_MSG_READ;
        if (index != 0u) {
            mapped |= FFL_XFER_MSG_RESTART;
        }
    }
    if ((flags & IMU_BUS_MSG_STOP) != 0u) {
        mapped |= FFL_XFER_MSG_STOP;
    }
    return mapped;
}

static int ffl_qmi8658a_xfer_adapter(void *ctx,
                                     const imu_bus_msg_t *msgs,
                                     uint8_t count,
                                     imu_bus_done_cb_t done,
                                     void *user)
{
    ffl_qmi8658a_device_t *device = (ffl_qmi8658a_device_t *)ctx;
    ffl_xfer_msg_t transport_msgs[2];
    ffl_endpoint_t endpoint;
    uint8_t index;

    (void)user;
    if (device == 0 || device->transport == 0 || msgs == 0 || count == 0u ||
        count > (uint8_t)(sizeof(transport_msgs) / sizeof(transport_msgs[0]))) {
        return -EINVAL;
    }
    if (done != 0) {
        return -ENOTSUP;
    }
    if (!ffl_transport_is_valid(device->transport) ||
        device->transport->endpoint.kind != FFL_ENDPOINT_I2C_7BIT ||
        device->addr == 0u || device->addr > 0x7Fu) {
        return -EINVAL;
    }

    endpoint = device->transport->endpoint;
    endpoint.value.i2c.addr7 = device->addr;
    for (index = 0u; index < count; ++index) {
        transport_msgs[index].buf = msgs[index].buf;
        transport_msgs[index].len = msgs[index].len;
        transport_msgs[index].flags = ffl_qmi8658a_map_flags(msgs[index].flags, index);
    }
    return ffl_transport_xfer(device->transport,
                              &endpoint,
                              transport_msgs,
                              count,
                              0,
                              0);
}

static void ffl_qmi8658a_delay_ms_adapter(void *ctx, uint32_t delay_ms)
{
    ffl_qmi8658a_device_t *device = (ffl_qmi8658a_device_t *)ctx;

    if (device != 0 && device->time_ops != 0 && device->time_ops->delay_ms != 0) {
        device->time_ops->delay_ms(device->time_ctx, delay_ms);
    }
}

static void ffl_qmi8658a_delay_us_adapter(void *ctx, uint32_t delay_us)
{
    ffl_qmi8658a_device_t *device = (ffl_qmi8658a_device_t *)ctx;

    if (device != 0 && device->time_ops != 0 && device->time_ops->delay_us != 0) {
        device->time_ops->delay_us(device->time_ctx, delay_us);
    }
}

static const imu_bus_ops_t g_ffl_qmi8658a_bus_ops = {
    .xfer = ffl_qmi8658a_xfer_adapter,
    .cancel = 0,
};

void ffl_qmi8658a_config_init(ffl_qmi8658a_config_t *config)
{
    if (config != 0) {
        *config = g_imu_qmi8658a_default_cfg;
    }
}

int ffl_qmi8658a_bind(ffl_qmi8658a_device_t *device,
                      const ffl_transport_t *transport,
                      const ffl_time_ops_t *time_ops,
                      void *time_ctx)
{
    int rc;

    if (device == 0 || !ffl_transport_is_valid(transport) ||
        transport->endpoint.kind != FFL_ENDPOINT_I2C_7BIT ||
        transport->endpoint.value.i2c.addr7 == 0u || time_ops == 0 ||
        time_ops->delay_ms == 0 || time_ops->delay_us == 0) {
        return -EINVAL;
    }
    rc = ffl_qmi8658a_try_lock(device);
    if (rc != 0) {
        return rc;
    }
    if (device->initialized) {
        ffl_qmi8658a_unlock(device);
        return -EBUSY;
    }

    device->bus_ops = &g_ffl_qmi8658a_bus_ops;
    device->bus_ctx = device;
    device->transport = transport;
    device->delay_ms = ffl_qmi8658a_delay_ms_adapter;
    device->delay_us = ffl_qmi8658a_delay_us_adapter;
    device->delay_ctx = device;
    device->time_ops = time_ops;
    device->time_ctx = time_ctx;
    device->addr = transport->endpoint.value.i2c.addr7;
    device->chip_id = 0u;
    device->initialized = false;
    device->cfg = g_imu_qmi8658a_default_cfg;
    ffl_qmi8658a_unlock(device);
    return 0;
}

int ffl_qmi8658a_set_i2c_addr(ffl_qmi8658a_device_t *device, uint8_t addr7)
{
    int rc;

    if (device == 0 || addr7 == 0u || addr7 > 0x7Fu) {
        return -EINVAL;
    }
    rc = ffl_qmi8658a_try_lock(device);
    if (rc != 0) {
        return rc;
    }
    if (device->initialized) {
        ffl_qmi8658a_unlock(device);
        return -EBUSY;
    }
    device->addr = addr7;
    ffl_qmi8658a_unlock(device);
    return 0;
}

int ffl_qmi8658a_init(ffl_qmi8658a_device_t *device,
                      const ffl_qmi8658a_config_t *config)
{
    int rc = ffl_qmi8658a_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = imu_qmi8658a_init(device, config);
    ffl_qmi8658a_unlock(device);
    return rc;
}

int ffl_qmi8658a_probe(ffl_qmi8658a_device_t *device, uint8_t *who_am_i)
{
    int rc = ffl_qmi8658a_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = imu_qmi8658a_probe(device, who_am_i);
    ffl_qmi8658a_unlock(device);
    return rc;
}

int ffl_qmi8658a_soft_reset(ffl_qmi8658a_device_t *device)
{
    int rc = ffl_qmi8658a_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = imu_qmi8658a_soft_reset(device);
    ffl_qmi8658a_unlock(device);
    return rc;
}

int ffl_qmi8658a_configure(ffl_qmi8658a_device_t *device,
                           const ffl_qmi8658a_config_t *config)
{
    int rc = ffl_qmi8658a_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    if (!device->initialized) {
        ffl_qmi8658a_unlock(device);
        return -ENODEV;
    }
    rc = imu_qmi8658a_configure(device, config);
    ffl_qmi8658a_unlock(device);
    return rc;
}

int ffl_qmi8658a_read_raw(ffl_qmi8658a_device_t *device,
                          ffl_qmi8658a_raw_sample_t *raw)
{
    int rc = ffl_qmi8658a_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = imu_qmi8658a_read_raw(device, raw);
    ffl_qmi8658a_unlock(device);
    return rc;
}

int ffl_qmi8658a_read_sample(ffl_qmi8658a_device_t *device,
                             ffl_qmi8658a_sample_t *sample)
{
    int rc = ffl_qmi8658a_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = imu_qmi8658a_read_sample(device, sample);
    ffl_qmi8658a_unlock(device);
    return rc;
}
