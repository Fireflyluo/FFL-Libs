#include "ffl/icm42688p.h"

#include "ffl_atomic.h"

#include <errno.h>

static int ffl_icm42688p_try_lock(ffl_icm42688p_device_t *device)
{
    if (device == NULL) {
        return -EINVAL;
    }
    return ffl_atomic_try_lock_u8(&device->in_use) ? 0 : -EBUSY;
}

static void ffl_icm42688p_unlock(ffl_icm42688p_device_t *device)
{
    if (device != NULL) {
        ffl_atomic_unlock_u8(&device->in_use);
    }
}

static uint8_t ffl_icm42688p_map_flags(uint8_t flags, uint8_t index)
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

static int ffl_icm42688p_xfer_adapter(void *ctx,
                                      const imu_bus_msg_t *msgs,
                                      uint8_t count,
                                      imu_bus_done_cb_t done,
                                      void *user)
{
    ffl_icm42688p_device_t *device = (ffl_icm42688p_device_t *)ctx;
    ffl_xfer_msg_t transport_msgs[2];
    ffl_endpoint_t endpoint;
    uint8_t index;

    (void)user;
    if (device == NULL || device->transport == NULL || msgs == NULL || count == 0u ||
        count > (uint8_t)(sizeof(transport_msgs) / sizeof(transport_msgs[0])) ||
        device->addr == 0u || device->addr > 0x7Fu ||
        !ffl_transport_is_valid(device->transport) ||
        device->transport->endpoint.kind != FFL_ENDPOINT_I2C_7BIT) {
        return -EINVAL;
    }
    if (done != NULL) {
        return -ENOTSUP;
    }

    endpoint = device->transport->endpoint;
    endpoint.value.i2c.addr7 = device->addr;
    for (index = 0u; index < count; ++index) {
        transport_msgs[index].buf = msgs[index].buf;
        transport_msgs[index].len = msgs[index].len;
        transport_msgs[index].flags = ffl_icm42688p_map_flags(msgs[index].flags, index);
    }
    return ffl_transport_xfer(device->transport, &endpoint, transport_msgs, count, NULL, NULL);
}

static void ffl_icm42688p_delay_ms_adapter(void *ctx, uint32_t delay_ms)
{
    ffl_icm42688p_device_t *device = (ffl_icm42688p_device_t *)ctx;

    if (device != NULL && device->time_ops != NULL && device->time_ops->delay_ms != NULL) {
        device->time_ops->delay_ms(device->time_ctx, delay_ms);
    }
}

static const imu_bus_ops_t ffl_icm42688p_bus_ops = {
    .xfer = ffl_icm42688p_xfer_adapter,
    .cancel = NULL,
};

void ffl_icm42688p_config_init(ffl_icm42688p_config_t *config)
{
    if (config != NULL) {
        *config = g_imu_icm42688p_default_cfg;
    }
}

int ffl_icm42688p_bind(ffl_icm42688p_device_t *device,
                       const ffl_transport_t *transport,
                       const ffl_time_ops_t *time_ops,
                       void *time_ctx)
{
    int rc;

    if (device == NULL || !ffl_transport_is_valid(transport) ||
        transport->endpoint.kind != FFL_ENDPOINT_I2C_7BIT ||
        transport->endpoint.value.i2c.addr7 == 0u ||
        transport->endpoint.value.i2c.addr7 > 0x7Fu ||
        time_ops == NULL || time_ops->delay_ms == NULL) {
        return -EINVAL;
    }
    rc = ffl_icm42688p_try_lock(device);
    if (rc != 0) {
        return rc;
    }
    if (device->initialized) {
        ffl_icm42688p_unlock(device);
        return -EBUSY;
    }
    device->bus_ops = &ffl_icm42688p_bus_ops;
    device->bus_ctx = device;
    device->transport = transport;
    device->time_ops = time_ops;
    device->time_ctx = time_ctx;
    device->delay_ms = ffl_icm42688p_delay_ms_adapter;
    device->delay_ctx = device;
    device->addr = transport->endpoint.value.i2c.addr7;
    device->chip_id = 0u;
    device->current_bank = (uint8_t)(ICM42688_BANK_MAX + 1);
    device->initialized = false;
    device->cfg = g_imu_icm42688p_default_cfg;
    ffl_icm42688p_unlock(device);
    return 0;
}

int ffl_icm42688p_set_i2c_addr(ffl_icm42688p_device_t *device, uint8_t addr7)
{
    int rc;

    if (device == NULL || addr7 == 0u || addr7 > 0x7Fu) {
        return -EINVAL;
    }
    rc = ffl_icm42688p_try_lock(device);
    if (rc != 0) {
        return rc;
    }
    if (device->initialized) {
        ffl_icm42688p_unlock(device);
        return -EBUSY;
    }
    device->addr = addr7;
    ffl_icm42688p_unlock(device);
    return 0;
}

int ffl_icm42688p_init(ffl_icm42688p_device_t *device,
                       const ffl_icm42688p_config_t *config)
{
    int rc = ffl_icm42688p_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = imu_icm42688p_init(device, config);
    ffl_icm42688p_unlock(device);
    return rc;
}

int ffl_icm42688p_probe(ffl_icm42688p_device_t *device, uint8_t *who_am_i)
{
    int rc = ffl_icm42688p_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = imu_icm42688p_probe(device, who_am_i);
    ffl_icm42688p_unlock(device);
    return rc;
}

int ffl_icm42688p_soft_reset(ffl_icm42688p_device_t *device)
{
    int rc = ffl_icm42688p_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = imu_icm42688p_soft_reset(device);
    ffl_icm42688p_unlock(device);
    return rc;
}

int ffl_icm42688p_configure(ffl_icm42688p_device_t *device,
                            const ffl_icm42688p_config_t *config)
{
    int rc = ffl_icm42688p_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    if (!device->initialized) {
        ffl_icm42688p_unlock(device);
        return -ENODEV;
    }
    rc = imu_icm42688p_configure(device, config);
    ffl_icm42688p_unlock(device);
    return rc;
}

int ffl_icm42688p_read_raw(ffl_icm42688p_device_t *device,
                           ffl_icm42688p_raw_sample_t *raw)
{
    int rc = ffl_icm42688p_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = imu_icm42688p_read_raw(device, raw);
    ffl_icm42688p_unlock(device);
    return rc;
}

int ffl_icm42688p_read_sample(ffl_icm42688p_device_t *device,
                              ffl_icm42688p_sample_t *sample)
{
    int rc = ffl_icm42688p_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = imu_icm42688p_read_sample(device, sample);
    ffl_icm42688p_unlock(device);
    return rc;
}
