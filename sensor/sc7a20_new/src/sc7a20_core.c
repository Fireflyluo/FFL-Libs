#include "../inc/sc7a20_core.h"

#include <errno.h>
#include <string.h>

#ifndef EHWPOISON
#define EHWPOISON 133
#endif

const sc7a20_cfg_t g_sc7a20_default_cfg = {
    .range = SC7A20_ACCEL_FS_2G,
    .odr = SC7A20_ACCEL_ODR_100HZ,
    .axis_x_en = true,
    .axis_y_en = true,
    .axis_z_en = true,
    .block_data_update = true,
    .high_resolution = false,
    .low_power = false,
};

static float sc7a20_sensitivity_g_per_lsb(sc7a20_accel_fs_t fs)
{
    switch (fs) {
    case SC7A20_ACCEL_FS_2G:
        return 0.0009765625f;
    case SC7A20_ACCEL_FS_4G:
        return 0.001953125f;
    case SC7A20_ACCEL_FS_8G:
        return 0.00390625f;
    case SC7A20_ACCEL_FS_16G:
        return 0.0078125f;
    default:
        return 0.0009765625f;
    }
}

int sc7a20_core_try_lock(sc7a20_dev_t *dev)
{
    if (dev == NULL) {
        return -EINVAL;
    }
    if (!__sync_bool_compare_and_swap(&dev->in_use, 0u, 1u)) {
        return -EBUSY;
    }
    return 0;
}

void sc7a20_core_unlock(sc7a20_dev_t *dev)
{
    if (dev != NULL) {
        __sync_lock_release(&dev->in_use);
    }
}

int sc7a20_core_map_bus_status(int status)
{
    if (status == 0) {
        return 0;
    }
    if (status < 0) {
        return status;
    }
    return -EIO;
}

int sc7a20_core_validate_dev(const sc7a20_dev_t *dev)
{
    if (dev == NULL || dev->ops == NULL || dev->ops->xfer == NULL) {
        return -EINVAL;
    }
    return 0;
}

static int sc7a20_core_xfer(sc7a20_dev_t *dev,
                            const sc7a20_comm_msg_t *msgs,
                            uint8_t cnt,
                            sc7a20_bus_done_cb_t cb,
                            void *user)
{
    const int valid = sc7a20_core_validate_dev(dev);
    if (valid != 0) {
        return valid;
    }
    return sc7a20_core_map_bus_status(dev->ops->xfer(dev->bus_ctx, msgs, cnt, cb, user));
}

int sc7a20_core_read_reg(sc7a20_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    sc7a20_comm_msg_t msgs[2];
    uint8_t reg_addr;

    if (data == NULL || len == 0u) {
        return -EINVAL;
    }

    reg_addr = reg;
    if (len > 1u) {
        reg_addr = (uint8_t)(reg_addr | 0x80u);
    }

    msgs[0].buf = &reg_addr;
    msgs[0].len = 1u;
    msgs[0].flags = SC7A20_COMM_WRITE;

    msgs[1].buf = data;
    msgs[1].len = len;
    msgs[1].flags = (uint8_t)(SC7A20_COMM_READ | SC7A20_COMM_STOP);

    return sc7a20_core_xfer(dev, msgs, 2u, NULL, NULL);
}

int sc7a20_core_write_reg(sc7a20_dev_t *dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    sc7a20_comm_msg_t msgs[2];
    uint8_t reg_addr;

    if (data == NULL || len == 0u) {
        return -EINVAL;
    }

    reg_addr = reg;
    if (len > 1u) {
        reg_addr = (uint8_t)(reg_addr | 0x80u);
    }

    msgs[0].buf = &reg_addr;
    msgs[0].len = 1u;
    msgs[0].flags = SC7A20_COMM_WRITE;

    msgs[1].buf = (uint8_t *)data;
    msgs[1].len = len;
    msgs[1].flags = (uint8_t)(SC7A20_COMM_WRITE | SC7A20_COMM_STOP);

    return sc7a20_core_xfer(dev, msgs, 2u, NULL, NULL);
}

int sc7a20_core_update_reg(sc7a20_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
    int rc;
    uint8_t old_v;
    uint8_t new_v;

    rc = sc7a20_core_read_reg(dev, reg, &old_v, 1u);
    if (rc != 0) {
        return rc;
    }

    new_v = (uint8_t)((old_v & (uint8_t)(~mask)) | (value & mask));
    if (new_v == old_v) {
        return 0;
    }

    return sc7a20_core_write_reg(dev, reg, &new_v, 1u);
}

int sc7a20_core_decode_xyz(const sc7a20_dev_t *dev, const uint8_t raw[6], sc7a20_vec3i16_t *out)
{
    int16_t x;
    int16_t y;
    int16_t z;

    if (dev == NULL || raw == NULL || out == NULL) {
        return -EINVAL;
    }

    if (dev->endian_ble == 0u) {
        x = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
        y = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
        z = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);
    } else {
        x = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
        y = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
        z = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
    }

    out->x = (int16_t)(x >> 4);
    out->y = (int16_t)(y >> 4);
    out->z = (int16_t)(z >> 4);
    return 0;
}

int sc7a20_core_apply_config(sc7a20_dev_t *dev, const sc7a20_cfg_t *cfg)
{
    int rc;
    sc7a20_ctrl0_t ctrl0;
    sc7a20_ctrl1_t ctrl1;
    sc7a20_ctrl2_t ctrl2;
    sc7a20_ctrl3_t ctrl3;
    sc7a20_ctrl4_t ctrl4;

    if (dev == NULL || cfg == NULL) {
        return -EINVAL;
    }

    memset(&ctrl0, 0, sizeof(ctrl0));
    memset(&ctrl1, 0, sizeof(ctrl1));
    memset(&ctrl2, 0, sizeof(ctrl2));
    memset(&ctrl3, 0, sizeof(ctrl3));
    memset(&ctrl4, 0, sizeof(ctrl4));

    ctrl0.bit.HR = cfg->high_resolution ? 1u : 0u;
    ctrl0.bit.OSR = 0u;

    ctrl1.bit.Xen = cfg->axis_x_en ? 1u : 0u;
    ctrl1.bit.Yen = cfg->axis_y_en ? 1u : 0u;
    ctrl1.bit.Zen = cfg->axis_z_en ? 1u : 0u;
    ctrl1.bit.LPen = cfg->low_power ? 1u : 0u;
    ctrl1.bit.ODR = (uint8_t)cfg->odr;

    ctrl2.reg = 0u;
    ctrl3.reg = 0u;

    ctrl4.reg = 0u;
    ctrl4.bit.BDU = cfg->block_data_update ? 1u : 0u;
    ctrl4.bit.fs = (uint8_t)cfg->range;

    rc = sc7a20_core_write_reg(dev, SC7A20_CTRL0, &ctrl0.reg, 1u);
    if (rc != 0) {
        return rc;
    }
    rc = sc7a20_core_write_reg(dev, SC7A20_CTRL1, &ctrl1.reg, 1u);
    if (rc != 0) {
        return rc;
    }
    rc = sc7a20_core_write_reg(dev, SC7A20_CTRL2, &ctrl2.reg, 1u);
    if (rc != 0) {
        return rc;
    }
    rc = sc7a20_core_write_reg(dev, SC7A20_CTRL3, &ctrl3.reg, 1u);
    if (rc != 0) {
        return rc;
    }
    rc = sc7a20_core_write_reg(dev, SC7A20_CTRL4, &ctrl4.reg, 1u);
    if (rc != 0) {
        return rc;
    }

    rc = sc7a20_core_read_reg(dev, SC7A20_CTRL4, &ctrl4.reg, 1u);
    if (rc != 0) {
        return rc;
    }

    dev->cfg = *cfg;
    dev->endian_ble = ctrl4.bit.BLE;
    dev->sensitivity_g_per_lsb = sc7a20_sensitivity_g_per_lsb(cfg->range);
    return 0;
}
