#include "icp20100.h"

#include <errno.h>
#include <string.h>

#ifndef EIO
#define EIO 5
#endif

#ifndef ENODEV
#define ENODEV 19
#endif

#ifndef EAGAIN
#define EAGAIN 11
#endif

const icp20100_cfg_t g_icp20100_default_cfg = {
    .addr = ICP20100_I2C_ADDR_AD0_LOW,
    .expected_chip_id = ICP20100_DEVICE_ID_DEFAULT,
    .op_mode = ICP20100_OP_MODE0,
    .meas_mode = ICP20100_MEAS_MODE_CONTINUOUS,
    .power_mode = ICP20100_POWER_MODE_NORMAL,
    .fifo_mode = ICP20100_FIFO_PRES_TEMP,
};

static int icp20100_validate(const icp20100_dev_t *dev)
{
    if (dev == NULL || dev->ops == NULL || dev->ops->xfer == NULL) {
        return -EINVAL;
    }
    if (dev->addr == 0u) {
        return -EINVAL;
    }
    return 0;
}

static int icp20100_map_status(int status)
{
    if (status == 0) {
        return 0;
    }
    if (status < 0) {
        return status;
    }
    return -EIO;
}

static int icp20100_xfer(icp20100_dev_t *dev, const icp20100_comm_msg_t *msgs, uint8_t cnt)
{
    const int rc = icp20100_validate(dev);
    if (rc != 0) {
        return rc;
    }
    return icp20100_map_status(dev->ops->xfer(dev->bus_ctx, msgs, cnt, NULL, NULL));
}

static void icp20100_delay_us(icp20100_dev_t *dev, uint32_t us)
{
    if (dev->delay_us != NULL) {
        dev->delay_us(dev->delay_ctx, us);
    }
}

static int32_t icp20100_sign_extend_20(uint32_t value20)
{
    int32_t out = (int32_t)(value20 & 0x0FFFFFu);
    if ((out & 0x080000) != 0) {
        out |= (int32_t)0xFFF00000;
    }
    return out;
}

int icp20100_read_reg(icp20100_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    icp20100_comm_msg_t msgs[2];
    uint8_t addr = reg;

    if (data == NULL || len == 0u) {
        return -EINVAL;
    }

    msgs[0].buf = &addr;
    msgs[0].len = 1u;
    msgs[0].flags = ICP20100_COMM_WRITE;

    msgs[1].buf = data;
    msgs[1].len = len;
    msgs[1].flags = (uint8_t)(ICP20100_COMM_READ | ICP20100_COMM_STOP);

    return icp20100_xfer(dev, msgs, 2u);
}

int icp20100_write_reg(icp20100_dev_t *dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    icp20100_comm_msg_t msgs[2];
    uint8_t addr = reg;

    if (data == NULL || len == 0u) {
        return -EINVAL;
    }

    msgs[0].buf = &addr;
    msgs[0].len = 1u;
    msgs[0].flags = ICP20100_COMM_WRITE;

    msgs[1].buf = (uint8_t *)data;
    msgs[1].len = len;
    msgs[1].flags = (uint8_t)(ICP20100_COMM_WRITE | ICP20100_COMM_STOP);

    return icp20100_xfer(dev, msgs, 2u);
}

int icp20100_probe(icp20100_dev_t *dev, uint8_t *chip_id, uint8_t *version)
{
    uint8_t id = 0u;
    uint8_t ver = 0u;
    int rc;

    if (dev == NULL) {
        return -EINVAL;
    }

    rc = icp20100_read_reg(dev, ICP20100_REG_DEVICE_ID, &id, 1u);
    if (rc != 0) {
        return rc;
    }
    rc = icp20100_read_reg(dev, ICP20100_REG_VERSION, &ver, 1u);
    if (rc != 0) {
        return rc;
    }

    dev->chip_id = id;
    dev->version = ver;
    if (chip_id != NULL) {
        *chip_id = id;
    }
    if (version != NULL) {
        *version = ver;
    }

    if (dev->cfg.expected_chip_id != 0u && id != dev->cfg.expected_chip_id) {
        return -ENODEV;
    }
    return 0;
}

int icp20100_soft_reset(icp20100_dev_t *dev)
{
    icp20100_mode_select_t mode;
    int rc;

    if (dev == NULL) {
        return -EINVAL;
    }

    memset(&mode, 0, sizeof(mode));
    rc = icp20100_write_reg(dev, ICP20100_REG_MODE_SELECT, &mode.reg, 1u);
    if (rc != 0) {
        return rc;
    }
    icp20100_delay_us(dev, 2000u);
    return 0;
}

int icp20100_set_config(icp20100_dev_t *dev, const icp20100_cfg_t *cfg)
{
    icp20100_cfg_t use_cfg;
    icp20100_mode_select_t mode;
    int rc;

    if (dev == NULL) {
        return -EINVAL;
    }

    use_cfg = (cfg != NULL) ? *cfg : dev->cfg;
    if (use_cfg.op_mode > ICP20100_OP_MODE4) {
        return -EINVAL;
    }

    memset(&mode, 0, sizeof(mode));
    mode.bit.FIFO_READOUT_MODE = (uint8_t)use_cfg.fifo_mode;
    mode.bit.POWER_MODE = (uint8_t)use_cfg.power_mode;
    mode.bit.MEAS_MODE = (uint8_t)use_cfg.meas_mode;
    mode.bit.FORCED_MEAS_TRIGGER = 0u;
    mode.bit.MEAS_CONFIG = (uint8_t)use_cfg.op_mode;

    rc = icp20100_write_reg(dev, ICP20100_REG_MODE_SELECT, &mode.reg, 1u);
    if (rc != 0) {
        return rc;
    }

    dev->cfg = use_cfg;
    return 0;
}

int icp20100_init(icp20100_dev_t *dev, const icp20100_cfg_t *cfg)
{
    icp20100_cfg_t local_cfg;
    uint8_t dummy = ICP20100_REG_DUMMY_VALUE;
    int rc;

    if (dev == NULL) {
        return -EINVAL;
    }

    local_cfg = (cfg != NULL) ? *cfg : g_icp20100_default_cfg;
    if (local_cfg.addr == 0u) {
        local_cfg.addr = ICP20100_I2C_ADDR_AD0_LOW;
    }

    dev->addr = local_cfg.addr;
    dev->cfg = local_cfg;

    rc = icp20100_write_reg(dev, ICP20100_REG_DUMMY_INIT, &dummy, 1u);
    if (rc != 0) {
        return rc;
    }
    icp20100_delay_us(dev, 50u);

    rc = icp20100_probe(dev, NULL, NULL);
    if (rc != 0) {
        return rc;
    }

    rc = icp20100_soft_reset(dev);
    if (rc != 0) {
        return rc;
    }

    rc = icp20100_set_config(dev, &local_cfg);
    if (rc != 0) {
        return rc;
    }

    dev->initialized = true;
    return 0;
}

int icp20100_read_raw(icp20100_dev_t *dev, icp20100_raw_sample_t *raw)
{
    icp20100_fifo_fill_t fifo_fill;
    uint8_t data[6];
    uint32_t press20;
    uint32_t temp20;
    int rc;

    if (dev == NULL || raw == NULL) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    rc = icp20100_read_reg(dev, ICP20100_REG_FIFO_FILL, &fifo_fill.reg, 1u);
    if (rc != 0) {
        return rc;
    }
    if (fifo_fill.bit.FIFO_LEVEL == 0u) {
        return -EAGAIN;
    }

    rc = icp20100_read_reg(dev, ICP20100_REG_FIFO_BASE, data, sizeof(data));
    if (rc != 0) {
        return rc;
    }

    press20 = ((uint32_t)(data[2] & 0x0Fu) << 16) | ((uint32_t)data[1] << 8) | data[0];
    temp20 = ((uint32_t)(data[5] & 0x0Fu) << 16) | ((uint32_t)data[4] << 8) | data[3];

    raw->pressure_raw = icp20100_sign_extend_20(press20);
    raw->temperature_raw = icp20100_sign_extend_20(temp20);
    return 0;
}

int icp20100_read_sample(icp20100_dev_t *dev, icp20100_sample_t *sample)
{
    icp20100_raw_sample_t raw;
    int rc;

    if (dev == NULL || sample == NULL) {
        return -EINVAL;
    }

    rc = icp20100_read_raw(dev, &raw);
    if (rc != 0) {
        return rc;
    }

    sample->pressure_kpa = ((float)raw.pressure_raw * 40.0f / 131072.0f) + 70.0f;
    sample->temperature_c = ((float)raw.temperature_raw * 65.0f / 262144.0f) + 25.0f;
    return 0;
}
