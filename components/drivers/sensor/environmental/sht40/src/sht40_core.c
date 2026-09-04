#include "../inc/sht40_core.h"

#include <errno.h>

int sht40_core_try_lock(sht40_dev_t *dev)
{
    if (dev == 0) {
        return -EINVAL;
    }
    if (!__sync_bool_compare_and_swap(&dev->in_use, 0u, 1u)) {
        return -EBUSY;
    }
    return 0;
}

void sht40_core_unlock(sht40_dev_t *dev)
{
    if (dev != 0) {
        __sync_lock_release(&dev->in_use);
    }
}

int sht40_core_validate_dev(const sht40_dev_t *dev)
{
    if (dev == 0 || dev->ops == 0 || dev->ops->xfer == 0) {
        return -EINVAL;
    }
    return 0;
}

int sht40_core_map_bus_status(int status)
{
    if (status == 0) {
        return 0;
    }
    if (status < 0) {
        return status;
    }
    return -EIO;
}

uint8_t sht40_core_precision_cmd(sht40_precision_t precision)
{
    switch (precision) {
    case SHT40_PRECISION_HIGH:
        return 0xFDu;
    case SHT40_PRECISION_MEDIUM:
        return 0xF6u;
    case SHT40_PRECISION_LOW:
        return 0xE0u;
    default:
        return 0xFDu;
    }
}

uint32_t sht40_core_measure_delay_ms(uint8_t cmd)
{
    if (cmd == 0xFDu) {
        return 10u;
    }
    if (cmd == 0xF6u) {
        return 6u;
    }
    if (cmd == 0xE0u) {
        return 3u;
    }
    if (cmd == 0x39u || cmd == 0x2Fu || cmd == 0x1Eu) {
        return 1200u;
    }
    if (cmd == 0x32u || cmd == 0x24u || cmd == 0x15u) {
        return 150u;
    }
    return 2u;
}

int sht40_core_xfer_sync(sht40_dev_t *dev, uint8_t *buf, uint16_t len, bool read)
{
    sht40_comm_msg_t msg;

    if (buf == 0 || len == 0u) {
        return -EINVAL;
    }

    msg.buf = buf;
    msg.len = len;
    msg.flags = (uint8_t)((read ? SHT40_COMM_READ : SHT40_COMM_WRITE) | SHT40_COMM_STOP);

    return sht40_core_map_bus_status(dev->ops->xfer(dev->bus_ctx, &msg, 1u, 0, 0));
}

int sht40_core_read_sample_parse(const uint8_t rx[6], sht40_sample_t *out)
{
    uint16_t t_raw;
    uint16_t h_raw;

    if (rx == 0 || out == 0) {
        return -EINVAL;
    }

    t_raw = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    h_raw = (uint16_t)(((uint16_t)rx[3] << 8) | rx[4]);

    out->temperature_c = -45.0f + 175.0f * ((float)t_raw / 65535.0f);
    out->humidity_rh = -6.0f + 125.0f * ((float)h_raw / 65535.0f);

    if (out->humidity_rh < 0.0f) {
        out->humidity_rh = 0.0f;
    }
    if (out->humidity_rh > 100.0f) {
        out->humidity_rh = 100.0f;
    }

    return 0;
}
