#include "ffl/qmc5883p.h"

#include "ffl_atomic.h"

#include <errno.h>
#include <string.h>

static int ffl_qmc5883p_try_lock(ffl_qmc5883p_device_t *device)
{
    if (device == 0) {
        return -EINVAL;
    }
    return ffl_atomic_try_lock_u8(&device->in_use) ? 0 : -EBUSY;
}

static void ffl_qmc5883p_unlock(ffl_qmc5883p_device_t *device)
{
    if (device != 0) {
        ffl_atomic_unlock_u8(&device->in_use);
    }
}

static uint8_t ffl_qmc5883p_map_flags(uint8_t flags)
{
    uint8_t mapped = 0u;

    if ((flags & QMC5883P_COMM_WRITE) != 0u) {
        mapped |= FFL_XFER_MSG_WRITE;
    }
    if ((flags & QMC5883P_COMM_READ) != 0u) {
        mapped |= FFL_XFER_MSG_READ;
    }
    if ((flags & QMC5883P_COMM_STOP) != 0u) {
        mapped |= FFL_XFER_MSG_STOP;
    }
    return mapped;
}

static int ffl_qmc5883p_xfer_adapter(void *ctx,
                                     const qmc5883p_comm_msg_t *msgs,
                                     uint8_t count,
                                     void *done_cb,
                                     void *user)
{
    qmc5883p_dev_t *device = (qmc5883p_dev_t *)ctx;
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
        transport_msgs[index].flags = ffl_qmc5883p_map_flags(msgs[index].flags);
    }
    return ffl_transport_xfer(device->transport, &endpoint, transport_msgs, count, 0, user);
}

static const qmc5883p_bus_ops_t g_ffl_qmc5883p_transport_adapter = {
    .xfer = ffl_qmc5883p_xfer_adapter,
};

void ffl_qmc5883p_config_init(ffl_qmc5883p_config_t *config)
{
    if (config != 0) {
        *config = g_qmc5883p_default_cfg;
    }
}

int ffl_qmc5883p_bind(ffl_qmc5883p_device_t *device,
                      const ffl_transport_t *transport)
{
    int rc;

    if (device == 0 || !ffl_transport_is_valid(transport) ||
        transport->endpoint.kind != FFL_ENDPOINT_I2C_7BIT ||
        transport->endpoint.value.i2c.addr7 == 0u) {
        return -EINVAL;
    }

    rc = ffl_qmc5883p_try_lock(device);
    if (rc != 0) {
        return rc;
    }
    if (device->initialized) {
        ffl_qmc5883p_unlock(device);
        return -EBUSY;
    }

    device->ops = &g_ffl_qmc5883p_transport_adapter;
    device->bus_ctx = device;
    device->transport = transport;
    device->addr = transport->endpoint.value.i2c.addr7;
    device->chip_id = 0u;
    memset(&device->cfg, 0, sizeof(device->cfg));
    ffl_qmc5883p_unlock(device);
    return 0;
}

int ffl_qmc5883p_set_i2c_addr(ffl_qmc5883p_device_t *device, uint8_t addr7)
{
    int rc;

    if (device == 0 || addr7 == 0u || addr7 > 0x7Fu) {
        return -EINVAL;
    }
    rc = ffl_qmc5883p_try_lock(device);
    if (rc != 0) {
        return rc;
    }
    if (device->initialized) {
        ffl_qmc5883p_unlock(device);
        return -EBUSY;
    }

    device->addr = addr7;
    ffl_qmc5883p_unlock(device);
    return 0;
}

int ffl_qmc5883p_init(ffl_qmc5883p_device_t *device,
                      const ffl_qmc5883p_config_t *config)
{
    ffl_qmc5883p_config_t local_config;
    int rc;

    if (device == 0) {
        return -EINVAL;
    }
    rc = ffl_qmc5883p_try_lock(device);
    if (rc != 0) {
        return rc;
    }
    if (config != 0) {
        local_config = *config;
    } else {
        local_config = g_qmc5883p_default_cfg;
        local_config.addr = device->addr;
    }

    rc = qmc5883p_init(device, &local_config);
    ffl_qmc5883p_unlock(device);
    return rc;
}

int ffl_qmc5883p_probe(ffl_qmc5883p_device_t *device, uint8_t *chip_id)
{
    int rc = ffl_qmc5883p_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = qmc5883p_probe(device, chip_id);
    ffl_qmc5883p_unlock(device);
    return rc;
}

int ffl_qmc5883p_soft_reset(ffl_qmc5883p_device_t *device)
{
    int rc = ffl_qmc5883p_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = qmc5883p_soft_reset(device);
    ffl_qmc5883p_unlock(device);
    return rc;
}

int ffl_qmc5883p_configure(ffl_qmc5883p_device_t *device,
                            const ffl_qmc5883p_config_t *config)
{
    int rc = ffl_qmc5883p_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = qmc5883p_set_config(device, config);
    ffl_qmc5883p_unlock(device);
    return rc;
}

int ffl_qmc5883p_read_raw(ffl_qmc5883p_device_t *device,
                           ffl_qmc5883p_vec3i16_t *sample)
{
    int rc = ffl_qmc5883p_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = qmc5883p_read_raw(device, sample);
    ffl_qmc5883p_unlock(device);
    return rc;
}

int ffl_qmc5883p_read_ut(ffl_qmc5883p_device_t *device,
                          ffl_qmc5883p_vec3f_t *sample)
{
    int rc = ffl_qmc5883p_try_lock(device);

    if (rc != 0) {
        return rc;
    }
    rc = qmc5883p_read_ut(device, sample);
    ffl_qmc5883p_unlock(device);
    return rc;
}
