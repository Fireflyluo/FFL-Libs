#include "../inc/sc7a20.h"

#include <errno.h>
#include <string.h>

int sc7a20_init(sc7a20_dev_t *dev)
{
    return sc7a20_init_with_config(dev, &g_sc7a20_default_cfg);
}

int sc7a20_init_with_config(sc7a20_dev_t *dev, const sc7a20_cfg_t *cfg)
{
    int rc;
    uint8_t who;
    const sc7a20_cfg_t *use_cfg;
    uint8_t reset_cmd;

    if (dev == NULL) {
        return -EINVAL;
    }

    rc = sc7a20_core_validate_dev(dev);
    if (rc != 0) {
        return rc;
    }

    rc = sc7a20_core_try_lock(dev);
    if (rc != 0) {
        return rc;
    }

    use_cfg = (cfg != NULL) ? cfg : &g_sc7a20_default_cfg;

    reset_cmd = 0xA5u;
    rc = sc7a20_core_write_reg(dev, SC7A20_SOFT_RESET, &reset_cmd, 1u);
    if (rc != 0) {
        sc7a20_core_unlock(dev);
        return rc;
    }

    rc = sc7a20_core_read_reg(dev, SC7A20_WHO_AM_I, &who, 1u);
    if (rc != 0) {
        sc7a20_core_unlock(dev);
        return rc;
    }

    if (who != SC7A20_CHIP_ID) {
        sc7a20_core_unlock(dev);
        return -ENODEV;
    }

    rc = sc7a20_core_apply_config(dev, use_cfg);
    if (rc != 0) {
        sc7a20_core_unlock(dev);
        return rc;
    }

    dev->who_am_i = who;
    dev->initialized = true;
    dev->async.op = SC7A20_ASYNC_OP_NONE;
    sc7a20_core_unlock(dev);
    return 0;
}

int sc7a20_deinit(sc7a20_dev_t *dev)
{
    int rc;
    sc7a20_ctrl1_t ctrl1;

    if (dev == NULL) {
        return -EINVAL;
    }
    rc = sc7a20_core_validate_dev(dev);
    if (rc != 0) {
        return rc;
    }

    rc = sc7a20_core_try_lock(dev);
    if (rc != 0) {
        return rc;
    }

    memset(&ctrl1, 0, sizeof(ctrl1));
    ctrl1.bit.ODR = SC7A20_ACCEL_ODR_POWER_DOWN;
    rc = sc7a20_core_write_reg(dev, SC7A20_CTRL1, &ctrl1.reg, 1u);
    dev->initialized = false;

    sc7a20_core_unlock(dev);
    return rc;
}

int sc7a20_soft_reset(sc7a20_dev_t *dev)
{
    int rc;
    uint8_t cmd;

    if (dev == NULL) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    rc = sc7a20_core_try_lock(dev);
    if (rc != 0) {
        return rc;
    }

    cmd = 0xA5u;
    rc = sc7a20_core_write_reg(dev, SC7A20_SOFT_RESET, &cmd, 1u);
    if (rc == 0) {
        rc = sc7a20_core_apply_config(dev, &dev->cfg);
    }

    sc7a20_core_unlock(dev);
    return rc;
}

int sc7a20_get_who_am_i(sc7a20_dev_t *dev, uint8_t *who_am_i)
{
    int rc;

    if (dev == NULL || who_am_i == NULL) {
        return -EINVAL;
    }
    rc = sc7a20_core_validate_dev(dev);
    if (rc != 0) {
        return rc;
    }

    rc = sc7a20_core_try_lock(dev);
    if (rc != 0) {
        return rc;
    }
    rc = sc7a20_core_read_reg(dev, SC7A20_WHO_AM_I, who_am_i, 1u);
    if (rc == 0) {
        dev->who_am_i = *who_am_i;
    }
    sc7a20_core_unlock(dev);
    return rc;
}

int sc7a20_read_reg(sc7a20_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    int rc;

    if (dev == NULL || data == NULL || len == 0u) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    rc = sc7a20_core_try_lock(dev);
    if (rc != 0) {
        return rc;
    }
    rc = sc7a20_core_read_reg(dev, reg, data, len);
    sc7a20_core_unlock(dev);
    return rc;
}

int sc7a20_write_reg(sc7a20_dev_t *dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    int rc;

    if (dev == NULL || data == NULL || len == 0u) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    rc = sc7a20_core_try_lock(dev);
    if (rc != 0) {
        return rc;
    }
    rc = sc7a20_core_write_reg(dev, reg, data, len);
    sc7a20_core_unlock(dev);
    return rc;
}

int sc7a20_read_xyz_raw(sc7a20_dev_t *dev, sc7a20_vec3i16_t *out)
{
    int rc;
    uint8_t raw[6];

    if (dev == NULL || out == NULL) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    rc = sc7a20_core_try_lock(dev);
    if (rc != 0) {
        return rc;
    }

    rc = sc7a20_core_read_reg(dev, SC7A20_OUTX_L, raw, 6u);
    if (rc == 0) {
        rc = sc7a20_core_decode_xyz(dev, raw, out);
    }

    sc7a20_core_unlock(dev);
    return rc;
}

int sc7a20_read_xyz_g(sc7a20_dev_t *dev, sc7a20_vec3f_t *out)
{
    int rc;
    sc7a20_vec3i16_t raw;

    if (dev == NULL || out == NULL) {
        return -EINVAL;
    }

    rc = sc7a20_read_xyz_raw(dev, &raw);
    if (rc != 0) {
        return rc;
    }

    out->x = raw.x * dev->sensitivity_g_per_lsb;
    out->y = raw.y * dev->sensitivity_g_per_lsb;
    out->z = raw.z * dev->sensitivity_g_per_lsb;
    return 0;
}

int sc7a20_set_range(sc7a20_dev_t *dev, sc7a20_accel_fs_t range)
{
    sc7a20_cfg_t cfg;

    if (dev == NULL) {
        return -EINVAL;
    }
    if (range > SC7A20_ACCEL_FS_16G) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    cfg = dev->cfg;
    cfg.range = range;

    if (sc7a20_core_try_lock(dev) != 0) {
        return -EBUSY;
    }
    const int rc = sc7a20_core_apply_config(dev, &cfg);
    sc7a20_core_unlock(dev);
    return rc;
}

int sc7a20_set_odr(sc7a20_dev_t *dev, sc7a20_accel_odr_t odr)
{
    sc7a20_cfg_t cfg;

    if (dev == NULL) {
        return -EINVAL;
    }
    if (odr > SC7A20_ACCEL_ODR_4_434KHZ) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    cfg = dev->cfg;
    cfg.odr = odr;

    if (sc7a20_core_try_lock(dev) != 0) {
        return -EBUSY;
    }
    const int rc = sc7a20_core_apply_config(dev, &cfg);
    sc7a20_core_unlock(dev);
    return rc;
}

int sc7a20_set_axis_enable(sc7a20_dev_t *dev, bool x_en, bool y_en, bool z_en)
{
    sc7a20_cfg_t cfg;

    if (dev == NULL) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    cfg = dev->cfg;
    cfg.axis_x_en = x_en;
    cfg.axis_y_en = y_en;
    cfg.axis_z_en = z_en;

    if (sc7a20_core_try_lock(dev) != 0) {
        return -EBUSY;
    }
    const int rc = sc7a20_core_apply_config(dev, &cfg);
    sc7a20_core_unlock(dev);
    return rc;
}

