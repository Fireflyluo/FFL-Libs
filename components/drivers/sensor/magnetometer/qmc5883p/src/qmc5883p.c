#include "qmc5883p.h"

#include <errno.h>
#include <string.h>

#ifndef EIO
#define EIO 5
#endif

#ifndef ENODEV
#define ENODEV 19
#endif

#define QMC5883P_I2C_ADDR_DEFAULT 0x2Cu

const qmc5883p_cfg_t g_qmc5883p_default_cfg = {
    .addr = QMC5883P_I2C_ADDR_DEFAULT,
    .expected_chip_id = QMC5883P_CHIP_ID_DEFAULT,
    .mode = QMC5883P_MODE_CONTINUOUS,
    .odr = QMC5883P_ODR_100HZ,
    .osr1 = QMC5883P_OSR1_8,
    .osr2 = QMC5883P_OSR2_1,
    .range = QMC5883P_RANGE_8G,
    .set_reset_mode = QMC5883P_SET_RESET_ON,
};

static int qmc5883p_validate(const qmc5883p_dev_t *dev)
{
    if (dev == NULL || dev->ops == NULL || dev->ops->xfer == NULL) {
        return -EINVAL;
    }
    if (dev->addr == 0u || dev->addr > 0x7Fu) {
        return -EINVAL;
    }
    return 0;
}

static int qmc5883p_map_status(int status)
{
    if (status == 0) {
        return 0;
    }
    if (status < 0) {
        return status;
    }
    return -EIO;
}

static int qmc5883p_xfer(qmc5883p_dev_t *dev, const qmc5883p_comm_msg_t *msgs, uint8_t cnt)
{
    const int rc = qmc5883p_validate(dev);
    if (rc != 0) {
        return rc;
    }
    return qmc5883p_map_status(dev->ops->xfer(dev->bus_ctx, msgs, cnt, NULL, NULL));
}

static float qmc5883p_lsb_per_ut(qmc5883p_range_t range)
{
    switch (range) {
    case QMC5883P_RANGE_2G:
        return 150.0f;
    case QMC5883P_RANGE_8G:
        return 37.5f;
    case QMC5883P_RANGE_12G:
        return 25.0f;
    case QMC5883P_RANGE_30G:
        return 10.0f;
    default:
        return 0.0f;
    }
}

int qmc5883p_read_reg(qmc5883p_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    qmc5883p_comm_msg_t msgs[2];
    uint8_t addr = reg;

    if (data == NULL || len == 0u) {
        return -EINVAL;
    }

    msgs[0].buf = &addr;
    msgs[0].len = 1u;
    msgs[0].flags = QMC5883P_COMM_WRITE;

    msgs[1].buf = data;
    msgs[1].len = len;
    msgs[1].flags = (uint8_t)(QMC5883P_COMM_READ | QMC5883P_COMM_STOP);

    return qmc5883p_xfer(dev, msgs, 2u);
}

int qmc5883p_write_reg(qmc5883p_dev_t *dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    qmc5883p_comm_msg_t msgs[2];
    uint8_t addr = reg;

    if (data == NULL || len == 0u) {
        return -EINVAL;
    }

    msgs[0].buf = &addr;
    msgs[0].len = 1u;
    msgs[0].flags = QMC5883P_COMM_WRITE;

    msgs[1].buf = (uint8_t *)data;
    msgs[1].len = len;
    msgs[1].flags = (uint8_t)(QMC5883P_COMM_WRITE | QMC5883P_COMM_STOP);

    return qmc5883p_xfer(dev, msgs, 2u);
}

int qmc5883p_probe(qmc5883p_dev_t *dev, uint8_t *chip_id)
{
    uint8_t id = 0u;
    int rc;

    if (dev == NULL) {
        return -EINVAL;
    }

    rc = qmc5883p_read_reg(dev, QMC5883P_REG_CHIP_ID, &id, 1u);
    if (rc != 0) {
        return rc;
    }

    dev->chip_id = id;
    if (chip_id != NULL) {
        *chip_id = id;
    }

    if (dev->cfg.expected_chip_id != 0u && id != dev->cfg.expected_chip_id) {
        return -ENODEV;
    }

    return 0;
}

int qmc5883p_soft_reset(qmc5883p_dev_t *dev)
{
    uint8_t ctrl2;
    int rc;

    if (dev == NULL) {
        return -EINVAL;
    }

    ctrl2 = QMC5883P_CTRL2_SOFT_RESET_MASK;
    ctrl2 |= (uint8_t)(((uint8_t)dev->cfg.range << QMC5883P_CTRL2_RNG_SHIFT) & QMC5883P_CTRL2_RNG_MASK);
    ctrl2 |= (uint8_t)((uint8_t)dev->cfg.set_reset_mode & QMC5883P_CTRL2_SET_RESET_MODE_MASK);

    rc = qmc5883p_write_reg(dev, QMC5883P_REG_CONTROL_2, &ctrl2, 1u);
    if (rc != 0) {
        return rc;
    }
    dev->initialized = false;
    return 0;
}

int qmc5883p_set_config(qmc5883p_dev_t *dev, const qmc5883p_cfg_t *cfg)
{
    qmc5883p_cfg_t use_cfg;
    uint8_t ctrl1;
    uint8_t ctrl2;
    int rc;

    if (dev == NULL) {
        return -EINVAL;
    }

    use_cfg = (cfg != NULL) ? *cfg : dev->cfg;
    if (use_cfg.addr == 0u) {
        use_cfg.addr = dev->addr;
    }
    if (use_cfg.addr == 0u || use_cfg.addr > 0x7Fu || use_cfg.addr != dev->addr ||
        use_cfg.mode > QMC5883P_MODE_CONTINUOUS || use_cfg.odr > QMC5883P_ODR_200HZ ||
        use_cfg.osr1 > QMC5883P_OSR1_1 || use_cfg.osr2 > QMC5883P_OSR2_8 ||
        use_cfg.range > QMC5883P_RANGE_2G ||
        use_cfg.set_reset_mode > QMC5883P_SET_RESET_OFF) {
        return -EINVAL;
    }

    if (dev->initialized) {
        ctrl1 = QMC5883P_MODE_SUSPEND;
        rc = qmc5883p_write_reg(dev, QMC5883P_REG_CONTROL_1, &ctrl1, 1u);
        if (rc != 0) {
            return rc;
        }
    }

    ctrl2 = (uint8_t)((uint8_t)use_cfg.set_reset_mode & QMC5883P_CTRL2_SET_RESET_MODE_MASK);
    ctrl2 |= (uint8_t)(((uint8_t)use_cfg.range << QMC5883P_CTRL2_RNG_SHIFT) & QMC5883P_CTRL2_RNG_MASK);
    rc = qmc5883p_write_reg(dev, QMC5883P_REG_CONTROL_2, &ctrl2, 1u);
    if (rc != 0) {
        return rc;
    }

    ctrl1 = (uint8_t)((uint8_t)use_cfg.mode & QMC5883P_CTRL1_MODE_MASK);
    ctrl1 |= (uint8_t)(((uint8_t)use_cfg.odr << QMC5883P_CTRL1_ODR_SHIFT) & QMC5883P_CTRL1_ODR_MASK);
    ctrl1 |= (uint8_t)(((uint8_t)use_cfg.osr1 << QMC5883P_CTRL1_OSR1_SHIFT) & QMC5883P_CTRL1_OSR1_MASK);
    ctrl1 |= (uint8_t)(((uint8_t)use_cfg.osr2 << QMC5883P_CTRL1_OSR2_SHIFT) & QMC5883P_CTRL1_OSR2_MASK);
    rc = qmc5883p_write_reg(dev, QMC5883P_REG_CONTROL_1, &ctrl1, 1u);
    if (rc != 0) {
        return rc;
    }

    dev->cfg = use_cfg;
    return 0;
}

int qmc5883p_init(qmc5883p_dev_t *dev, const qmc5883p_cfg_t *cfg)
{
    qmc5883p_cfg_t local_cfg;
    int rc;

    if (dev == NULL) {
        return -EINVAL;
    }

    local_cfg = (cfg != NULL) ? *cfg : g_qmc5883p_default_cfg;
    if (local_cfg.addr == 0u) {
        local_cfg.addr = QMC5883P_I2C_ADDR_DEFAULT;
    }
    if (local_cfg.addr > 0x7Fu) {
        return -EINVAL;
    }

    dev->initialized = false;

    dev->addr = local_cfg.addr;
    dev->cfg = local_cfg;

    rc = qmc5883p_probe(dev, NULL);
    if (rc != 0) {
        return rc;
    }

    rc = qmc5883p_soft_reset(dev);
    if (rc != 0) {
        return rc;
    }

    rc = qmc5883p_set_config(dev, &local_cfg);
    if (rc != 0) {
        return rc;
    }

    dev->initialized = true;
    return 0;
}

int qmc5883p_read_raw(qmc5883p_dev_t *dev, qmc5883p_vec3i16_t *out)
{
    uint8_t buf[6];
    uint8_t status;
    int rc;

    if (dev == NULL || out == NULL) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    rc = qmc5883p_read_reg(dev, QMC5883P_REG_STATUS, &status, 1u);
    if (rc != 0) {
        return rc;
    }
    if ((status & QMC5883P_STATUS_OVFL_MASK) != 0u) {
        return -EIO;
    }
    if ((status & QMC5883P_STATUS_DRDY_MASK) == 0u) {
        return -EAGAIN;
    }

    rc = qmc5883p_read_reg(dev, QMC5883P_REG_XOUT_L, buf, sizeof(buf));
    if (rc != 0) {
        return rc;
    }

    out->x = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    out->y = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    out->z = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    return 0;
}

int qmc5883p_read_ut(qmc5883p_dev_t *dev, qmc5883p_vec3f_t *out)
{
    qmc5883p_vec3i16_t raw;
    float scale;
    int rc;

    if (dev == NULL || out == NULL) {
        return -EINVAL;
    }

    rc = qmc5883p_read_raw(dev, &raw);
    if (rc != 0) {
        return rc;
    }

    scale = 1.0f / qmc5883p_lsb_per_ut(dev->cfg.range);
    out->x = (float)raw.x * scale;
    out->y = (float)raw.y * scale;
    out->z = (float)raw.z * scale;
    return 0;
}
