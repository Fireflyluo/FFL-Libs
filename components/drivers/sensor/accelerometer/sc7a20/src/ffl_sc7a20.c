#include "ffl/sc7a20.h"
#include "sc7a20.h"

#include <errno.h>
#include <string.h>

static uint8_t ffl_sc7a20_map_flags(uint8_t flags, uint8_t index)
{
    uint8_t mapped = 0u;

    if ((flags & SC7A20_COMM_WRITE) != 0u) {
        mapped |= FFL_XFER_MSG_WRITE;
    }
    if ((flags & SC7A20_COMM_READ) != 0u) {
        mapped |= FFL_XFER_MSG_READ;
        if (index != 0u) {
            mapped |= FFL_XFER_MSG_RESTART;
        }
    }
    if ((flags & SC7A20_COMM_STOP) != 0u) {
        mapped |= FFL_XFER_MSG_STOP;
    }
    return mapped;
}

static int ffl_sc7a20_xfer_adapter(void *ctx,
                                   const sc7a20_comm_msg_t *msgs,
                                   uint8_t count,
                                   sc7a20_bus_done_cb_t done,
                                   void *user)
{
    ffl_sc7a20_device_t *device = (ffl_sc7a20_device_t *)ctx;
    ffl_xfer_msg_t transport_msgs[2];
    ffl_endpoint_t endpoint;
    uint8_t index;

    if (device == NULL || device->transport == NULL || msgs == NULL || count == 0u ||
        count > (uint8_t)(sizeof(transport_msgs) / sizeof(transport_msgs[0]))) {
        return -EINVAL;
    }
    if (!ffl_transport_is_valid(device->transport)) {
        return -EINVAL;
    }
    if (device->transport->endpoint.kind != FFL_ENDPOINT_I2C_7BIT) {
        return -ENOTSUP;
    }
    if (device->core.addr == 0u || device->core.addr > 0x7Fu) {
        return -EINVAL;
    }

    endpoint = device->transport->endpoint;
    endpoint.value.i2c.addr7 = device->core.addr;
    for (index = 0u; index < count; ++index) {
        transport_msgs[index].buf = msgs[index].buf;
        transport_msgs[index].len = msgs[index].len;
        transport_msgs[index].flags = ffl_sc7a20_map_flags(msgs[index].flags, index);
    }

    return ffl_transport_xfer(device->transport,
                              &endpoint,
                              transport_msgs,
                              count,
                              done,
                              user);
}

static int ffl_sc7a20_cancel_adapter(void *ctx)
{
    const ffl_sc7a20_device_t *device = (const ffl_sc7a20_device_t *)ctx;

    if (device == NULL || device->transport == NULL) {
        return -EINVAL;
    }
    return ffl_transport_cancel(device->transport);
}

static const sc7a20_bus_ops_t g_ffl_sc7a20_bus_ops = {
    .xfer = ffl_sc7a20_xfer_adapter,
    .cancel = ffl_sc7a20_cancel_adapter,
};

static int ffl_sc7a20_validate_config(const ffl_sc7a20_config_t *config)
{
    if (config == NULL) {
        return 0;
    }
    if (config->range > SC7A20_ACCEL_FS_16G ||
        config->odr > SC7A20_ACCEL_ODR_4_434KHZ) {
        return -EINVAL;
    }
    return 0;
}

void ffl_sc7a20_config_init(ffl_sc7a20_config_t *config)
{
    if (config != NULL) {
        *config = g_sc7a20_default_cfg;
    }
}

int ffl_sc7a20_bind(ffl_sc7a20_device_t *device,
                    const ffl_transport_t *transport,
                    const ffl_time_ops_t *time_ops,
                    void *time_ctx)
{
    int rc;

    if (device == NULL || !ffl_transport_is_valid(transport)) {
        return -EINVAL;
    }
    if (transport->endpoint.kind != FFL_ENDPOINT_I2C_7BIT) {
        return -ENOTSUP;
    }
    if (transport->endpoint.value.i2c.addr7 == 0u ||
        transport->endpoint.value.i2c.addr7 > 0x7Fu) {
        return -EINVAL;
    }

    rc = sc7a20_core_try_lock(&device->core);
    if (rc != 0) {
        return rc;
    }
    if (device->core.initialized) {
        sc7a20_core_unlock(&device->core);
        return -EBUSY;
    }

    device->transport = transport;
    device->time_ops = time_ops;
    device->time_ctx = time_ctx;
    device->core.ops = &g_ffl_sc7a20_bus_ops;
    device->core.bus_ctx = device;
    device->core.addr = transport->endpoint.value.i2c.addr7;
    device->core.cfg = g_sc7a20_default_cfg;
    device->core.initialized = false;
    device->core.who_am_i = 0u;
    device->core.endian_ble = 0u;
    device->core.sensitivity_g_per_lsb = 0.0009765625f;
    memset(&device->core.async, 0, sizeof(device->core.async));
    sc7a20_core_unlock(&device->core);
    return 0;
}

int ffl_sc7a20_set_i2c_addr(ffl_sc7a20_device_t *device, uint8_t addr7)
{
    int rc;

    if (device == NULL || addr7 == 0u || addr7 > 0x7Fu) {
        return -EINVAL;
    }
    rc = sc7a20_core_try_lock(&device->core);
    if (rc != 0) {
        return rc;
    }
    if (device->core.initialized) {
        sc7a20_core_unlock(&device->core);
        return -EBUSY;
    }
    device->core.addr = addr7;
    sc7a20_core_unlock(&device->core);
    return 0;
}

int ffl_sc7a20_init(ffl_sc7a20_device_t *device,
                    const ffl_sc7a20_config_t *config)
{
    if (device == NULL) {
        return -EINVAL;
    }
    return ffl_sc7a20_validate_config(config) == 0
               ? sc7a20_init_with_config(&device->core, config)
               : -EINVAL;
}

int ffl_sc7a20_deinit(ffl_sc7a20_device_t *device)
{
    if (device == NULL) {
        return -EINVAL;
    }
    return sc7a20_deinit(&device->core);
}

int ffl_sc7a20_soft_reset(ffl_sc7a20_device_t *device)
{
    if (device == NULL) {
        return -EINVAL;
    }
    return sc7a20_soft_reset(&device->core);
}

int ffl_sc7a20_who_am_i(ffl_sc7a20_device_t *device, uint8_t *who_am_i)
{
    if (device == NULL) {
        return -EINVAL;
    }
    return sc7a20_get_who_am_i(&device->core, who_am_i);
}

int ffl_sc7a20_read_raw(ffl_sc7a20_device_t *device,
                        ffl_sc7a20_raw_t *raw)
{
    if (device == NULL) {
        return -EINVAL;
    }
    return sc7a20_read_xyz_raw(&device->core, raw);
}

int ffl_sc7a20_read_g(ffl_sc7a20_device_t *device,
                      ffl_sc7a20_g_t *sample)
{
    if (device == NULL) {
        return -EINVAL;
    }
    return sc7a20_read_xyz_g(&device->core, sample);
}

int ffl_sc7a20_set_range(ffl_sc7a20_device_t *device,
                         ffl_sc7a20_range_t range)
{
    if (device == NULL) {
        return -EINVAL;
    }
    return sc7a20_set_range(&device->core, range);
}

int ffl_sc7a20_set_odr(ffl_sc7a20_device_t *device,
                       ffl_sc7a20_odr_t odr)
{
    if (device == NULL) {
        return -EINVAL;
    }
    return sc7a20_set_odr(&device->core, odr);
}

int ffl_sc7a20_set_axis_enable(ffl_sc7a20_device_t *device,
                               bool x_en,
                               bool y_en,
                               bool z_en)
{
    if (device == NULL) {
        return -EINVAL;
    }
    return sc7a20_set_axis_enable(&device->core, x_en, y_en, z_en);
}

#if FFL_SC7A20_ASYNC_ENABLED
int ffl_sc7a20_read_reg_async(ffl_sc7a20_device_t *device,
                              uint8_t reg,
                              uint8_t *data,
                              uint16_t len,
                              ffl_sc7a20_done_fn callback,
                              void *user)
{
    if (device == NULL) {
        return -EINVAL;
    }
    return sc7a20_read_reg_async(&device->core, reg, data, len, callback, user);
}

int ffl_sc7a20_write_reg_async(ffl_sc7a20_device_t *device,
                               uint8_t reg,
                               const uint8_t *data,
                               uint16_t len,
                               ffl_sc7a20_done_fn callback,
                               void *user)
{
    if (device == NULL) {
        return -EINVAL;
    }
    return sc7a20_write_reg_async(&device->core, reg, data, len, callback, user);
}

int ffl_sc7a20_read_raw_async(ffl_sc7a20_device_t *device,
                              ffl_sc7a20_read_raw_fn callback,
                              void *user)
{
    if (device == NULL) {
        return -EINVAL;
    }
    return sc7a20_read_xyz_raw_async(&device->core, callback, user);
}

int ffl_sc7a20_cancel_async(ffl_sc7a20_device_t *device)
{
    if (device == NULL) {
        return -EINVAL;
    }
    return sc7a20_cancel_async(&device->core);
}
#endif
