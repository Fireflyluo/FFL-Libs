#include "imu_qmi8658a.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#ifndef EIO
#define EIO 5
#endif

#ifndef ENODEV
#define ENODEV 19
#endif

#define QMI8658A_CTRL1_ADDR_AI_MASK   0x40u
#define QMI8658A_CTRL2_ODR_MASK       0x0Fu
#define QMI8658A_CTRL2_FS_MASK        0x70u
#define QMI8658A_CTRL2_FS_SHIFT       4u
#define QMI8658A_CTRL3_ODR_MASK       0x0Fu
#define QMI8658A_CTRL3_FS_MASK        0x70u
#define QMI8658A_CTRL3_FS_SHIFT       4u
#define QMI8658A_CTRL5_ACCEL_LPF_EN   0x01u
#define QMI8658A_CTRL5_ACCEL_LPF_MASK 0x06u
#define QMI8658A_CTRL5_ACCEL_LPF_SHIFT 1u
#define QMI8658A_CTRL5_GYRO_LPF_EN    0x10u
#define QMI8658A_CTRL5_GYRO_LPF_MASK  0x60u
#define QMI8658A_CTRL5_GYRO_LPF_SHIFT 5u
#define QMI8658A_CTRL7_ACCEL_EN       0x01u
#define QMI8658A_CTRL7_GYRO_EN        0x02u
#define QMI8658A_CTRL7_SYNC_SAMPLE    0x80u
#define QMI8658A_DATA_LOCK_DELAY_MAX_US 12u
#define QMI8658A_CTRL8_CTRL9_HANDSHAKE_STATUS 0x80u
#define QMI8658A_CMD_DONE_RETRIES 4u

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const imu_qmi8658a_cfg_t g_imu_qmi8658a_default_cfg = {
    .accel_fs = QMI8658A_ACCEL_FS_4G,
    .accel_odr = QMI8658A_ACCEL_ODR_112_1HZ,
    .gyro_fs = QMI8658A_GYRO_FS_512DPS,
    .gyro_odr = QMI8658A_GYRO_ODR_112_1HZ,
    .enable_accel = true,
    .enable_gyro = true,
    .enable_auto_increment = true,
    .enable_sync_sample = true,
    .accel_lpf_enable = false,
    .accel_lpf_mode = 0u,
    .gyro_lpf_enable = false,
    .gyro_lpf_mode = 0u,
};

static int imu_qmi8658a_validate(const imu_qmi8658a_t *dev)
{
    if (dev == NULL || dev->bus_ops == NULL || dev->bus_ops->xfer == NULL) {
        return -EINVAL;
    }
    if (dev->addr == 0u || dev->addr > 0x7Fu) {
        return -EINVAL;
    }
    return 0;
}

static int imu_qmi8658a_validate_config(const imu_qmi8658a_cfg_t *cfg)
{
    if (cfg == 0 || !cfg->enable_accel || !cfg->enable_gyro ||
        !cfg->enable_auto_increment ||
        cfg->accel_fs > QMI8658A_ACCEL_FS_16G ||
        cfg->accel_odr > QMI8658A_ACCEL_ODR_28_025HZ ||
        cfg->gyro_fs > QMI8658A_GYRO_FS_2048DPS ||
        cfg->gyro_odr > QMI8658A_GYRO_ODR_56_05HZ ||
        cfg->accel_lpf_mode > 3u || cfg->gyro_lpf_mode > 3u) {
        return -EINVAL;
    }
    return 0;
}

static int imu_qmi8658a_map_status(int status)
{
    if (status == 0) {
        return 0;
    }
    if (status < 0) {
        return status;
    }
    return -EIO;
}

static void imu_qmi8658a_delay_us(imu_qmi8658a_t *dev, uint32_t us);

static int imu_qmi8658a_set_ahb_clock_gating(imu_qmi8658a_t *dev, bool enabled)
{
    uint8_t cal1 = enabled ? 0u : 1u;
    uint8_t ctrl8 = QMI8658A_CTRL8_CTRL9_HANDSHAKE_STATUS;
    uint8_t command = QMI8658A_CTRL9_CMD_AHB_CLOCK_GATING;
    uint8_t status = 0u;
    uint8_t attempt;
    int rc;

    rc = imu_qmi8658a_write_reg(dev, QMI8658A_CAL1_L, &cal1, 1u);
    if (rc != 0) {
        return rc;
    }
    rc = imu_qmi8658a_write_reg(dev, QMI8658A_CTRL8, &ctrl8, 1u);
    if (rc != 0) {
        return rc;
    }
    rc = imu_qmi8658a_write_reg(dev, QMI8658A_CTRL9, &command, 1u);
    if (rc != 0) {
        return rc;
    }
    for (attempt = 0u; attempt < QMI8658A_CMD_DONE_RETRIES; ++attempt) {
        rc = imu_qmi8658a_read_reg(dev, QMI8658A_STATUSINT, &status, 1u);
        if (rc != 0) {
            return rc;
        }
        if ((status & QMI8658A_STATUSINT_CMD_DONE_MASK) != 0u) {
            command = QMI8658A_CTRL9_CMD_ACK;
            rc = imu_qmi8658a_write_reg(dev, QMI8658A_CTRL9, &command, 1u);
            if (rc != 0) {
                return rc;
            }
            rc = imu_qmi8658a_read_reg(dev, QMI8658A_STATUSINT, &status, 1u);
            if (rc != 0) {
                return rc;
            }
            return (status & QMI8658A_STATUSINT_CMD_DONE_MASK) == 0u ? 0 : -EIO;
        }
        imu_qmi8658a_delay_us(dev, 1u);
    }
    return -ETIMEDOUT;
}

static int imu_qmi8658a_xfer(imu_qmi8658a_t *dev,
                             const imu_bus_msg_t *msgs,
                             uint8_t cnt)
{
    const int rc = imu_qmi8658a_validate(dev);
    if (rc != 0) {
        return rc;
    }

    return imu_qmi8658a_map_status(dev->bus_ops->xfer(dev->bus_ctx, msgs, cnt, NULL, NULL));
}

static void imu_qmi8658a_delay(imu_qmi8658a_t *dev, uint32_t ms)
{
    if (dev->delay_ms != NULL) {
        dev->delay_ms(dev->delay_ctx, ms);
    }
}

static void imu_qmi8658a_delay_us(imu_qmi8658a_t *dev, uint32_t us)
{
    if (dev->delay_us != 0) {
        dev->delay_us(dev->delay_ctx, us);
    }
}

static uint32_t imu_qmi8658a_lock_delay_us(qmi8658a_gyro_odr_t odr)
{
    switch (odr) {
    case QMI8658A_GYRO_ODR_7174_4HZ:
    case QMI8658A_GYRO_ODR_3587_2HZ:
        return 2u;
    case QMI8658A_GYRO_ODR_1793_6HZ:
        return 4u;
    case QMI8658A_GYRO_ODR_896_8HZ:
        return 6u;
    default:
        return QMI8658A_DATA_LOCK_DELAY_MAX_US;
    }
}

static float imu_qmi8658a_accel_g(qmi8658a_accel_fs_t fs)
{
    switch (fs) {
    case QMI8658A_ACCEL_FS_2G:
        return 2.0f;
    case QMI8658A_ACCEL_FS_4G:
        return 4.0f;
    case QMI8658A_ACCEL_FS_8G:
        return 8.0f;
    case QMI8658A_ACCEL_FS_16G:
        return 16.0f;
    default:
        return 4.0f;
    }
}

static float imu_qmi8658a_gyro_dps(qmi8658a_gyro_fs_t fs)
{
    switch (fs) {
    case QMI8658A_GYRO_FS_16DPS:
        return 16.0f;
    case QMI8658A_GYRO_FS_32DPS:
        return 32.0f;
    case QMI8658A_GYRO_FS_64DPS:
        return 64.0f;
    case QMI8658A_GYRO_FS_128DPS:
        return 128.0f;
    case QMI8658A_GYRO_FS_256DPS:
        return 256.0f;
    case QMI8658A_GYRO_FS_512DPS:
        return 512.0f;
    case QMI8658A_GYRO_FS_1024DPS:
        return 1024.0f;
    case QMI8658A_GYRO_FS_2048DPS:
        return 2048.0f;
    default:
        return 512.0f;
    }
}

int imu_qmi8658a_read_reg(imu_qmi8658a_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    imu_bus_msg_t msgs[2];
    uint8_t reg_addr = reg;

    if (data == NULL || len == 0u) {
        return -EINVAL;
    }

    msgs[0].buf = &reg_addr;
    msgs[0].len = 1u;
    msgs[0].flags = IMU_BUS_MSG_WRITE;

    msgs[1].buf = data;
    msgs[1].len = len;
    msgs[1].flags = (uint8_t)(IMU_BUS_MSG_READ | IMU_BUS_MSG_STOP);

    return imu_qmi8658a_xfer(dev, msgs, 2u);
}

int imu_qmi8658a_write_reg(imu_qmi8658a_t *dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    imu_bus_msg_t msgs[2];
    uint8_t reg_addr = reg;

    if (data == NULL || len == 0u) {
        return -EINVAL;
    }

    msgs[0].buf = &reg_addr;
    msgs[0].len = 1u;
    msgs[0].flags = IMU_BUS_MSG_WRITE;

    msgs[1].buf = (uint8_t *)data;
    msgs[1].len = len;
    msgs[1].flags = (uint8_t)(IMU_BUS_MSG_WRITE | IMU_BUS_MSG_STOP);

    return imu_qmi8658a_xfer(dev, msgs, 2u);
}

int imu_qmi8658a_probe(imu_qmi8658a_t *dev, uint8_t *who_am_i)
{
    uint8_t id = 0u;
    int rc;

    rc = imu_qmi8658a_read_reg(dev, QMI8658A_WHO_AM_I, &id, 1u);
    if (rc != 0) {
        return rc;
    }

    dev->chip_id = id;
    if (who_am_i != NULL) {
        *who_am_i = id;
    }

    return (id == QMI8658A_ID) ? 0 : -ENODEV;
}

int imu_qmi8658a_soft_reset(imu_qmi8658a_t *dev)
{
    uint8_t cmd = 0xB0u;
    int rc;

    if (dev == 0 || dev->delay_ms == 0) {
        return -EINVAL;
    }
    rc = imu_qmi8658a_write_reg(dev, QMI8658A_RESET, &cmd, 1u);
    if (rc != 0) {
        dev->initialized = false;
        return rc;
    }

    imu_qmi8658a_delay(dev, 20u);
    dev->initialized = false;
    return 0;
}

int imu_qmi8658a_set_accel_config(imu_qmi8658a_t *dev,
                                  qmi8658a_accel_fs_t fs,
                                  qmi8658a_accel_odr_t odr)
{
    uint8_t ctrl2;
    int rc;

    if (dev == 0 || fs > QMI8658A_ACCEL_FS_16G ||
        odr > QMI8658A_ACCEL_ODR_28_025HZ) {
        return -EINVAL;
    }
    ctrl2 = (uint8_t)((uint8_t)odr & QMI8658A_CTRL2_ODR_MASK);
    ctrl2 |= (uint8_t)(((uint8_t)fs << QMI8658A_CTRL2_FS_SHIFT) & QMI8658A_CTRL2_FS_MASK);
    rc = imu_qmi8658a_write_reg(dev, QMI8658A_CTRL2, &ctrl2, 1u);
    if (rc == 0) {
        dev->cfg.accel_fs = fs;
        dev->cfg.accel_odr = odr;
    }
    return rc;
}

int imu_qmi8658a_set_gyro_config(imu_qmi8658a_t *dev,
                                 qmi8658a_gyro_fs_t fs,
                                 qmi8658a_gyro_odr_t odr)
{
    uint8_t ctrl3;
    int rc;

    if (dev == 0 || fs > QMI8658A_GYRO_FS_2048DPS ||
        odr > QMI8658A_GYRO_ODR_56_05HZ) {
        return -EINVAL;
    }
    ctrl3 = (uint8_t)((uint8_t)odr & QMI8658A_CTRL3_ODR_MASK);
    ctrl3 |= (uint8_t)(((uint8_t)fs << QMI8658A_CTRL3_FS_SHIFT) & QMI8658A_CTRL3_FS_MASK);
    rc = imu_qmi8658a_write_reg(dev, QMI8658A_CTRL3, &ctrl3, 1u);
    if (rc == 0) {
        dev->cfg.gyro_fs = fs;
        dev->cfg.gyro_odr = odr;
    }
    return rc;
}

static int imu_qmi8658a_write_config(imu_qmi8658a_t *dev,
                                     const imu_qmi8658a_cfg_t *cfg)
{
    uint8_t ctrl1 = QMI8658A_CTRL1_ADDR_AI_MASK;
    uint8_t ctrl2;
    uint8_t ctrl3;
    uint8_t ctrl5;
    uint8_t ctrl7 = 0u;
    uint8_t ctrl7_without_sync;
    int rc;

    ctrl2 = (uint8_t)((uint8_t)cfg->accel_odr & QMI8658A_CTRL2_ODR_MASK);
    ctrl2 |= (uint8_t)(((uint8_t)cfg->accel_fs << QMI8658A_CTRL2_FS_SHIFT) & QMI8658A_CTRL2_FS_MASK);
    ctrl3 = (uint8_t)((uint8_t)cfg->gyro_odr & QMI8658A_CTRL3_ODR_MASK);
    ctrl3 |= (uint8_t)(((uint8_t)cfg->gyro_fs << QMI8658A_CTRL3_FS_SHIFT) & QMI8658A_CTRL3_FS_MASK);
    ctrl5 = cfg->accel_lpf_enable ? QMI8658A_CTRL5_ACCEL_LPF_EN : 0u;
    ctrl5 |= (uint8_t)(((uint8_t)cfg->accel_lpf_mode << QMI8658A_CTRL5_ACCEL_LPF_SHIFT) & QMI8658A_CTRL5_ACCEL_LPF_MASK);
    ctrl5 |= cfg->gyro_lpf_enable ? QMI8658A_CTRL5_GYRO_LPF_EN : 0u;
    ctrl5 |= (uint8_t)(((uint8_t)cfg->gyro_lpf_mode << QMI8658A_CTRL5_GYRO_LPF_SHIFT) & QMI8658A_CTRL5_GYRO_LPF_MASK);
    ctrl7 |= cfg->enable_accel ? QMI8658A_CTRL7_ACCEL_EN : 0u;
    ctrl7 |= cfg->enable_gyro ? QMI8658A_CTRL7_GYRO_EN : 0u;
    ctrl7 |= cfg->enable_sync_sample ? QMI8658A_CTRL7_SYNC_SAMPLE : 0u;
    ctrl7_without_sync = (uint8_t)(ctrl7 & (uint8_t)~QMI8658A_CTRL7_SYNC_SAMPLE);

    if (dev->initialized && dev->cfg.enable_sync_sample) {
        rc = imu_qmi8658a_write_reg(dev, QMI8658A_CTRL7, &ctrl7_without_sync, 1u);
        if (rc != 0) {
            return rc;
        }
        imu_qmi8658a_delay(dev, 1u);
        rc = imu_qmi8658a_read_reg(dev, QMI8658A_OUTZ_H_G, &ctrl1, 1u);
        if (rc != 0) {
            return rc;
        }
    }

    rc = imu_qmi8658a_write_reg(dev, QMI8658A_CTRL1, &ctrl1, 1u);
    if (rc != 0) {
        return rc;
    }
    rc = imu_qmi8658a_write_reg(dev, QMI8658A_CTRL2, &ctrl2, 1u);
    if (rc != 0) {
        return rc;
    }
    rc = imu_qmi8658a_write_reg(dev, QMI8658A_CTRL3, &ctrl3, 1u);
    if (rc != 0) {
        return rc;
    }
    rc = imu_qmi8658a_write_reg(dev, QMI8658A_CTRL5, &ctrl5, 1u);
    if (rc != 0) {
        return rc;
    }
    rc = imu_qmi8658a_write_reg(dev, QMI8658A_CTRL7, &ctrl7_without_sync, 1u);
    if (rc != 0) {
        return rc;
    }
    rc = imu_qmi8658a_set_ahb_clock_gating(dev, !cfg->enable_sync_sample);
    if (rc != 0) {
        return rc;
    }
    if (!cfg->enable_sync_sample) {
        return 0;
    }
    return imu_qmi8658a_write_reg(dev, QMI8658A_CTRL7, &ctrl7, 1u);
}

int imu_qmi8658a_configure(imu_qmi8658a_t *dev,
                           const imu_qmi8658a_cfg_t *cfg)
{
    imu_qmi8658a_cfg_t next;
    int rc;

    if (dev == 0) {
        return -EINVAL;
    }
    next = cfg != 0 ? *cfg : dev->cfg;
    rc = imu_qmi8658a_validate_config(&next);
    if (rc != 0) {
        return rc;
    }
    rc = imu_qmi8658a_write_config(dev, &next);
    if (rc != 0) {
        dev->initialized = false;
        return rc;
    }
    dev->cfg = next;
    return 0;
}

int imu_qmi8658a_init(imu_qmi8658a_t *dev, const imu_qmi8658a_cfg_t *cfg)
{
    imu_qmi8658a_cfg_t local_cfg;
    int rc;

    if (dev == NULL) {
        return -EINVAL;
    }

    dev->initialized = false;
    local_cfg = (cfg != NULL) ? *cfg : g_imu_qmi8658a_default_cfg;
    rc = imu_qmi8658a_validate(dev);
    if (rc != 0) {
        return rc;
    }
    rc = imu_qmi8658a_validate_config(&local_cfg);
    if (rc != 0) {
        return rc;
    }

    rc = imu_qmi8658a_probe(dev, NULL);
    if (rc != 0) {
        return rc;
    }

    rc = imu_qmi8658a_soft_reset(dev);
    if (rc != 0) {
        return rc;
    }

    rc = imu_qmi8658a_write_config(dev, &local_cfg);
    if (rc != 0) {
        return rc;
    }

    dev->cfg = local_cfg;
    dev->initialized = true;
    return 0;
}

int imu_qmi8658a_read_raw(imu_qmi8658a_t *dev, imu_raw_sample_t *raw)
{
    uint8_t statusint;
    uint8_t status0;
    uint8_t buf[14];
    int rc;

    if (dev == NULL || raw == NULL) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    if (dev->cfg.enable_sync_sample) {
        rc = imu_qmi8658a_read_reg(dev, QMI8658A_STATUSINT, &statusint, 1u);
        if (rc != 0) {
            return rc;
        }
        if ((statusint & QMI8658A_STATUSINT_AVAIL_MASK) == 0u) {
            return -EAGAIN;
        }
        if ((statusint & QMI8658A_STATUSINT_LOCKED_MASK) == 0u) {
            imu_qmi8658a_delay_us(dev, imu_qmi8658a_lock_delay_us(dev->cfg.gyro_odr));
            rc = imu_qmi8658a_read_reg(dev, QMI8658A_STATUSINT, &statusint, 1u);
            if (rc != 0) {
                return rc;
            }
            if ((statusint & QMI8658A_STATUSINT_LOCKED_MASK) == 0u) {
                return -EAGAIN;
            }
        }
    }

    rc = imu_qmi8658a_read_reg(dev, QMI8658A_STATUS0, &status0, 1u);
    if (rc != 0) {
        return rc;
    }
    if ((status0 & (QMI8658A_STATUS0_ADA_MASK | QMI8658A_STATUS0_GDA_MASK)) !=
        (QMI8658A_STATUS0_ADA_MASK | QMI8658A_STATUS0_GDA_MASK)) {
        return -EAGAIN;
    }

    rc = imu_qmi8658a_read_reg(dev, QMI8658A_OUT_TEMP_L, buf, sizeof(buf));
    if (rc != 0) {
        return rc;
    }

    raw->temperature = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    raw->accel[0] = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    raw->accel[1] = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    raw->accel[2] = (int16_t)(((uint16_t)buf[7] << 8) | buf[6]);
    raw->gyro[0] = (int16_t)(((uint16_t)buf[9] << 8) | buf[8]);
    raw->gyro[1] = (int16_t)(((uint16_t)buf[11] << 8) | buf[10]);
    raw->gyro[2] = (int16_t)(((uint16_t)buf[13] << 8) | buf[12]);
    return 0;
}

int imu_qmi8658a_read_sample(imu_qmi8658a_t *dev, imu_sample_t *sample)
{
    imu_raw_sample_t raw;
    float accel_scale;
    float gyro_scale;
    int rc;
    int i;

    if (sample == NULL) {
        return -EINVAL;
    }

    rc = imu_qmi8658a_read_raw(dev, &raw);
    if (rc != 0) {
        return rc;
    }

    accel_scale = (imu_qmi8658a_accel_g(dev->cfg.accel_fs) / 32768.0f) * 9.80665f;
    gyro_scale = (imu_qmi8658a_gyro_dps(dev->cfg.gyro_fs) / 32768.0f) * ((float)M_PI / 180.0f);

    for (i = 0; i < 3; ++i) {
        sample->accel_mps2[i] = (float)raw.accel[i] * accel_scale;
        sample->gyro_rads[i] = (float)raw.gyro[i] * gyro_scale;
    }
    sample->temperature_c = (float)raw.temperature / 256.0f;
    sample->timestamp_ms = 0u;
    return 0;
}
