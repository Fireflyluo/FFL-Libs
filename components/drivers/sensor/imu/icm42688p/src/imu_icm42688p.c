#include "imu_icm42688p.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#ifndef EIO
#define EIO 5
#endif

#ifndef ENODEV
#define ENODEV 19
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define IMU_ICM42688P_DEFAULT_ADDR   0x69u
#define IMU_ICM42688P_EXPECTED_ID    0x47u
#define ICM42688_DEVICE_CONFIG_SOFT_RESET_MASK 0x01u
#define ICM42688_INTF_CONFIG1_CLKSEL_MASK       0x03u
#define ICM42688_PWR_MGMT0_ACCEL_MODE_MASK      0x03u
#define ICM42688_PWR_MGMT0_GYRO_MODE_MASK       0x0Cu
#define ICM42688_PWR_MGMT0_GYRO_MODE_SHIFT      2u
#define ICM42688_GYRO_CONFIG0_ODR_MASK          0x0Fu
#define ICM42688_GYRO_CONFIG0_FS_MASK           0xE0u
#define ICM42688_GYRO_CONFIG0_FS_SHIFT          5u
#define ICM42688_ACCEL_CONFIG0_ODR_MASK         0x0Fu
#define ICM42688_ACCEL_CONFIG0_FS_MASK          0xE0u
#define ICM42688_ACCEL_CONFIG0_FS_SHIFT         5u

const imu_icm42688p_cfg_t g_imu_icm42688p_default_cfg = {
    .addr = IMU_ICM42688P_DEFAULT_ADDR,
    .accel_mode = ICM42688_MODE_LOW_NOISE,
    .gyro_mode = ICM42688_MODE_LOW_NOISE,
    .accel_fs = ICM42688_ACCEL_FS_4G,
    .accel_odr = ICM42688_ODR_100HZ,
    .gyro_fs = ICM42688_GYRO_FS_500DPS,
    .gyro_odr = ICM42688_ODR_100HZ,
};

static int imu_icm42688p_validate(const imu_icm42688p_t *dev)
{
    if (dev == NULL || dev->bus_ops == NULL || dev->bus_ops->xfer == NULL) {
        return -EINVAL;
    }
    if (dev->addr == 0u) {
        return -EINVAL;
    }
    return 0;
}

static int imu_icm42688p_map_status(int status)
{
    if (status == 0) {
        return 0;
    }
    if (status < 0) {
        return status;
    }
    return -EIO;
}

static int imu_icm42688p_xfer(imu_icm42688p_t *dev,
                              const imu_bus_msg_t *msgs,
                              uint8_t cnt)
{
    const int rc = imu_icm42688p_validate(dev);
    if (rc != 0) {
        return rc;
    }

    return imu_icm42688p_map_status(dev->bus_ops->xfer(dev->bus_ctx, msgs, cnt, NULL, NULL));
}

static void imu_icm42688p_delay(imu_icm42688p_t *dev, uint32_t ms)
{
    if (dev->delay_ms != NULL) {
        dev->delay_ms(dev->delay_ctx, ms);
    }
}

static float imu_icm42688p_accel_lsb_per_g(icm42688_accel_fs_t fs)
{
    switch (fs) {
    case ICM42688_ACCEL_FS_16G:
        return 2048.0f;
    case ICM42688_ACCEL_FS_8G:
        return 4096.0f;
    case ICM42688_ACCEL_FS_4G:
        return 8192.0f;
    case ICM42688_ACCEL_FS_2G:
        return 16384.0f;
    default:
        return 8192.0f;
    }
}

static float imu_icm42688p_gyro_lsb_per_dps(icm42688_gyro_fs_t fs)
{
    switch (fs) {
    case ICM42688_GYRO_FS_2000DPS:
        return 16.384f;
    case ICM42688_GYRO_FS_1000DPS:
        return 32.768f;
    case ICM42688_GYRO_FS_500DPS:
        return 65.536f;
    case ICM42688_GYRO_FS_250DPS:
        return 131.072f;
    case ICM42688_GYRO_FS_125DPS:
        return 262.144f;
    case ICM42688_GYRO_FS_62_5DPS:
        return 524.288f;
    case ICM42688_GYRO_FS_31_25DPS:
        return 1048.576f;
    case ICM42688_GYRO_FS_15_625DPS:
        return 2097.152f;
    default:
        return 65.536f;
    }
}

static int imu_icm42688p_switch_bank(imu_icm42688p_t *dev, icm42688_bank_t bank)
{
    imu_bus_msg_t msgs[2];
    uint8_t bank_sel;
    uint8_t reg_addr = ICM42688_REG_BANK_SEL;

    if (bank > ICM42688_BANK_MAX) {
        return -EINVAL;
    }
    if (dev->current_bank == (uint8_t)bank) {
        return 0;
    }

    bank_sel = (uint8_t)bank;
    msgs[0].buf = &reg_addr;
    msgs[0].len = 1u;
    msgs[0].flags = IMU_BUS_MSG_WRITE;

    msgs[1].buf = &bank_sel;
    msgs[1].len = 1u;
    msgs[1].flags = (uint8_t)(IMU_BUS_MSG_WRITE | IMU_BUS_MSG_STOP);

    if (imu_icm42688p_xfer(dev, msgs, 2u) != 0) {
        return -EIO;
    }

    dev->current_bank = (uint8_t)bank;
    return 0;
}

int imu_icm42688p_read_reg(imu_icm42688p_t *dev,
                           icm42688_bank_t bank,
                           uint8_t reg,
                           uint8_t *data,
                           uint16_t len)
{
    imu_bus_msg_t msgs[2];
    int rc;
    uint8_t reg_addr = reg;

    if (data == NULL || len == 0u) {
        return -EINVAL;
    }

    rc = imu_icm42688p_switch_bank(dev, bank);
    if (rc != 0) {
        return rc;
    }

    msgs[0].buf = &reg_addr;
    msgs[0].len = 1u;
    msgs[0].flags = IMU_BUS_MSG_WRITE;

    msgs[1].buf = data;
    msgs[1].len = len;
    msgs[1].flags = (uint8_t)(IMU_BUS_MSG_READ | IMU_BUS_MSG_STOP);

    return imu_icm42688p_xfer(dev, msgs, 2u);
}

int imu_icm42688p_write_reg(imu_icm42688p_t *dev,
                            icm42688_bank_t bank,
                            uint8_t reg,
                            const uint8_t *data,
                            uint16_t len)
{
    imu_bus_msg_t msgs[2];
    int rc;
    uint8_t reg_addr = reg;

    if (data == NULL || len == 0u) {
        return -EINVAL;
    }

    rc = imu_icm42688p_switch_bank(dev, bank);
    if (rc != 0) {
        return rc;
    }

    msgs[0].buf = &reg_addr;
    msgs[0].len = 1u;
    msgs[0].flags = IMU_BUS_MSG_WRITE;

    msgs[1].buf = (uint8_t *)data;
    msgs[1].len = len;
    msgs[1].flags = (uint8_t)(IMU_BUS_MSG_WRITE | IMU_BUS_MSG_STOP);

    return imu_icm42688p_xfer(dev, msgs, 2u);
}

int imu_icm42688p_probe(imu_icm42688p_t *dev, uint8_t *who_am_i)
{
    uint8_t id = 0u;
    int rc;

    rc = imu_icm42688p_read_reg(dev, ICM42688_BANK0, ICM42688_REG_WHO_AM_I, &id, 1u);
    if (rc != 0) {
        return rc;
    }

    dev->chip_id = id;
    if (who_am_i != NULL) {
        *who_am_i = id;
    }

    return (id == IMU_ICM42688P_EXPECTED_ID) ? 0 : -ENODEV;
}

int imu_icm42688p_soft_reset(imu_icm42688p_t *dev)
{
    uint8_t cfg = ICM42688_DEVICE_CONFIG_SOFT_RESET_MASK;
    int rc;

    rc = imu_icm42688p_write_reg(dev, ICM42688_BANK0, ICM42688_REG_DEVICE_CONFIG, &cfg, 1u);
    if (rc != 0) {
        return rc;
    }

    dev->current_bank = (uint8_t)ICM42688_BANK0;
    imu_icm42688p_delay(dev, 2u);
    return 0;
}

int imu_icm42688p_set_gyro_config(imu_icm42688p_t *dev,
                                  icm42688_gyro_fs_t fs,
                                  icm42688_odr_t odr)
{
    uint8_t cfg = (uint8_t)((uint8_t)odr & ICM42688_GYRO_CONFIG0_ODR_MASK);
    cfg |= (uint8_t)(((uint8_t)fs << ICM42688_GYRO_CONFIG0_FS_SHIFT) & ICM42688_GYRO_CONFIG0_FS_MASK);

    dev->cfg.gyro_fs = fs;
    dev->cfg.gyro_odr = odr;
    return imu_icm42688p_write_reg(dev, ICM42688_BANK0, ICM42688_REG_GYRO_CONFIG0, &cfg, 1u);
}

int imu_icm42688p_set_accel_config(imu_icm42688p_t *dev,
                                   icm42688_accel_fs_t fs,
                                   icm42688_odr_t odr)
{
    uint8_t cfg = (uint8_t)((uint8_t)odr & ICM42688_ACCEL_CONFIG0_ODR_MASK);
    cfg |= (uint8_t)(((uint8_t)fs << ICM42688_ACCEL_CONFIG0_FS_SHIFT) & ICM42688_ACCEL_CONFIG0_FS_MASK);

    dev->cfg.accel_fs = fs;
    dev->cfg.accel_odr = odr;
    return imu_icm42688p_write_reg(dev, ICM42688_BANK0, ICM42688_REG_ACCEL_CONFIG0, &cfg, 1u);
}

int imu_icm42688p_init(imu_icm42688p_t *dev, const imu_icm42688p_cfg_t *cfg)
{
    imu_icm42688p_cfg_t local_cfg;
    uint8_t intf_cfg;
    uint8_t pwr_cfg;
    int rc;

    if (dev == NULL) {
        return -EINVAL;
    }

    local_cfg = (cfg != NULL) ? *cfg : g_imu_icm42688p_default_cfg;
    if (local_cfg.addr == 0u) {
        local_cfg.addr = IMU_ICM42688P_DEFAULT_ADDR;
    }

    dev->addr = local_cfg.addr;
    dev->current_bank = (uint8_t)ICM42688_BANK0;

    rc = imu_icm42688p_probe(dev, NULL);
    if (rc != 0) {
        return rc;
    }

    rc = imu_icm42688p_soft_reset(dev);
    if (rc != 0) {
        return rc;
    }

    intf_cfg = 0x01u & ICM42688_INTF_CONFIG1_CLKSEL_MASK;
    rc = imu_icm42688p_write_reg(dev, ICM42688_BANK0, ICM42688_REG_INTF_CONFIG1, &intf_cfg, 1u);
    if (rc != 0) {
        return rc;
    }

    pwr_cfg = (uint8_t)((uint8_t)local_cfg.accel_mode & ICM42688_PWR_MGMT0_ACCEL_MODE_MASK);
    pwr_cfg |= (uint8_t)(((uint8_t)local_cfg.gyro_mode << ICM42688_PWR_MGMT0_GYRO_MODE_SHIFT) & ICM42688_PWR_MGMT0_GYRO_MODE_MASK);
    rc = imu_icm42688p_write_reg(dev, ICM42688_BANK0, ICM42688_REG_PWR_MGMT0, &pwr_cfg, 1u);
    if (rc != 0) {
        return rc;
    }

    imu_icm42688p_delay(dev, 10u);

    rc = imu_icm42688p_set_gyro_config(dev, local_cfg.gyro_fs, local_cfg.gyro_odr);
    if (rc != 0) {
        return rc;
    }

    rc = imu_icm42688p_set_accel_config(dev, local_cfg.accel_fs, local_cfg.accel_odr);
    if (rc != 0) {
        return rc;
    }

    dev->cfg = local_cfg;
    dev->initialized = true;
    return 0;
}

int imu_icm42688p_read_raw(imu_icm42688p_t *dev, imu_raw_sample_t *raw)
{
    uint8_t buf[14];
    int rc;

    if (dev == NULL || raw == NULL) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -EINVAL;
    }

    rc = imu_icm42688p_read_reg(dev, ICM42688_BANK0, ICM42688_REG_TEMP_DATA1, buf, sizeof(buf));
    if (rc != 0) {
        return rc;
    }

    raw->temperature = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    raw->accel[0] = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    raw->accel[1] = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    raw->accel[2] = (int16_t)(((uint16_t)buf[6] << 8) | buf[7]);
    raw->gyro[0] = (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
    raw->gyro[1] = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    raw->gyro[2] = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);
    return 0;
}

int imu_icm42688p_read_sample(imu_icm42688p_t *dev, imu_sample_t *sample)
{
    imu_raw_sample_t raw;
    float accel_scale;
    float gyro_scale;
    int rc;
    int i;

    if (sample == NULL) {
        return -EINVAL;
    }

    rc = imu_icm42688p_read_raw(dev, &raw);
    if (rc != 0) {
        return rc;
    }

    accel_scale = (1.0f / imu_icm42688p_accel_lsb_per_g(dev->cfg.accel_fs)) * 9.80665f;
    gyro_scale = (1.0f / imu_icm42688p_gyro_lsb_per_dps(dev->cfg.gyro_fs)) * ((float)M_PI / 180.0f);

    for (i = 0; i < 3; ++i) {
        sample->accel_mps2[i] = (float)raw.accel[i] * accel_scale;
        sample->gyro_rads[i] = (float)raw.gyro[i] * gyro_scale;
    }
    sample->temperature_c = ((float)raw.temperature / 132.48f) + 25.0f;
    sample->timestamp_ms = 0u;
    return 0;
}
