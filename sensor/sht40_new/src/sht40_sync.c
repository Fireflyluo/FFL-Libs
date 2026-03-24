#include "../inc/sht40.h"

#include <errno.h>
#include <string.h>

static void sht40_delay_if_present(const sht40_dev_t *dev, uint32_t ms)
{
    if (dev->delay_ms != 0) {
        dev->delay_ms(dev->delay_ctx, ms);
    }
}

int sht40_init(sht40_dev_t *dev)
{
    int rc;

    if (dev == 0) {
        return -EINVAL;
    }

    rc = sht40_core_validate_dev(dev);
    if (rc != 0) {
        return rc;
    }

    rc = sht40_core_try_lock(dev);
    if (rc != 0) {
        return rc;
    }

    dev->addr = (dev->addr == 0u) ? SHT40_I2C_ADDR : dev->addr;
    memset(&dev->async, 0, sizeof(dev->async));
    dev->initialized = true;

    rc = sht40_soft_reset(dev);

    sht40_core_unlock(dev);
    return rc;
}

int sht40_soft_reset(sht40_dev_t *dev)
{
    int rc;
    uint8_t cmd = 0x94u;

    if (dev == 0) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    rc = sht40_core_xfer_sync(dev, &cmd, 1u, false);
    if (rc != 0) {
        return rc;
    }

    sht40_delay_if_present(dev, 2u);
    return 0;
}

int sht40_read_serial(sht40_dev_t *dev, uint32_t *serial)
{
    int rc;
    uint8_t cmd = 0x89u;
    uint8_t rx[6];

    if (dev == 0 || serial == 0) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    rc = sht40_core_xfer_sync(dev, &cmd, 1u, false);
    if (rc != 0) {
        return rc;
    }

    sht40_delay_if_present(dev, 1u);

    rc = sht40_core_xfer_sync(dev, rx, 6u, true);
    if (rc != 0) {
        return rc;
    }

    *serial = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) | ((uint32_t)rx[3] << 8) | rx[4];
    return 0;
}

int sht40_read_sample(sht40_dev_t *dev, sht40_precision_t precision, sht40_sample_t *out)
{
    int rc;
    uint8_t cmd;
    uint8_t rx[6];

    if (dev == 0 || out == 0) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    cmd = sht40_core_precision_cmd(precision);

    rc = sht40_core_xfer_sync(dev, &cmd, 1u, false);
    if (rc != 0) {
        return rc;
    }

    sht40_delay_if_present(dev, sht40_core_measure_delay_ms(cmd));

    rc = sht40_core_xfer_sync(dev, rx, 6u, true);
    if (rc != 0) {
        return rc;
    }

    return sht40_core_read_sample_parse(rx, out);
}

int sht40_heater(sht40_dev_t *dev, sht40_heater_cmd_t cmd)
{
    int rc;
    uint8_t b;

    if (dev == 0) {
        return -EINVAL;
    }
    if (!dev->initialized) {
        return -ENODEV;
    }

    b = (uint8_t)cmd;
    rc = sht40_core_xfer_sync(dev, &b, 1u, false);
    if (rc != 0) {
        return rc;
    }

    sht40_delay_if_present(dev, sht40_core_measure_delay_ms(b));
    return 0;
}
