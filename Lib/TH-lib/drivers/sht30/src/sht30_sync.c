#include "sht30.h"

#include <errno.h>
#include <string.h>

static void sht30_delay_if_present(const sht30_dev_t *dev, uint32_t ms)
{
    if (dev->delay_ms != 0) {
        dev->delay_ms(dev->delay_ctx, ms);
    }
}

int sht30_init(sht30_dev_t *dev)
{
    int rc;

    if (dev == 0) {
        return -EINVAL;
    }

    rc = sht30_core_validate_dev(dev);
    if (rc != 0) {
        return rc;
    }

    rc = sht30_core_try_lock(dev);
    if (rc != 0) {
        return rc;
    }

    dev->addr = (dev->addr == 0u) ? SHT30_I2C_ADDR : dev->addr;
    memset(&dev->async, 0, sizeof(dev->async));
    dev->initialized = true;

    rc = sht30_soft_reset(dev);

    sht30_core_unlock(dev);
    return rc;
}

int sht30_soft_reset(sht30_dev_t *dev)
{
    int rc;
    uint8_t cmd[2] = {0x30u, 0xA2u};

    if (dev == 0) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    rc = sht30_core_xfer_sync(dev, cmd, 2u, false);
    if (rc != 0) {
        return rc;
    }

    sht30_delay_if_present(dev, 2u);
    return 0;
}

int sht30_read_status(sht30_dev_t *dev, uint16_t *status)
{
    int rc;
    uint8_t cmd[2] = {0xF3u, 0x2Du};
    uint8_t rx[3];

    if (dev == 0 || status == 0) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    rc = sht30_core_xfer_sync(dev, cmd, 2u, false);
    if (rc != 0) {
        return rc;
    }

    sht30_delay_if_present(dev, 1u);

    rc = sht30_core_xfer_sync(dev, rx, 3u, true);
    if (rc != 0) {
        return rc;
    }

    *status = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    return 0;
}

int sht30_read_sample(sht30_dev_t *dev, sht30_repeatability_t repeatability, sht30_sample_t *out)
{
    int rc;
    uint8_t cmd[2];
    uint8_t rx[6];

    if (dev == 0 || out == 0) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    sht30_core_precision_cmd(cmd, repeatability);

    rc = sht30_core_xfer_sync(dev, cmd, 2u, false);
    if (rc != 0) {
        return rc;
    }

    sht30_delay_if_present(dev, sht30_core_measure_delay_ms(cmd));

    rc = sht30_core_xfer_sync(dev, rx, 6u, true);
    if (rc != 0) {
        return rc;
    }

    return sht30_core_read_sample_parse(rx, out);
}

int sht30_heater(sht30_dev_t *dev, sht30_heater_cmd_t cmd)
{
    int rc;
    uint8_t b[2];

    if (dev == 0) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    b[0] = (uint8_t)((uint16_t)cmd >> 8);
    b[1] = (uint8_t)((uint16_t)cmd & 0xFFu);
    rc = sht30_core_xfer_sync(dev, b, 2u, false);
    if (rc != 0) {
        return rc;
    }

    sht30_delay_if_present(dev, 1u);
    return 0;
}