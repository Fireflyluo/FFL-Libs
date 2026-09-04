#include "sht30_core.h"

#include <errno.h>

#ifndef EBADMSG
#define EBADMSG EIO
#endif

static uint8_t sht30_core_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFFu;
    uint8_t index;
    uint8_t bit;

    for (index = 0u; index < len; ++index) {
        crc ^= data[index];
        for (bit = 0u; bit < 8u; ++bit) {
            crc = (crc & 0x80u) != 0u ? (uint8_t)((crc << 1) ^ 0x31u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}
#if defined(_MSC_VER)
#include <intrin.h>
#endif

static int sht30_core_lock_cas(volatile uint8_t *ptr, uint8_t expected, uint8_t desired)
{
#if defined(_MSC_VER)
    return _InterlockedCompareExchange8((volatile char *)ptr, (char)desired, (char)expected) == (char)expected;
#else
    return __sync_bool_compare_and_swap(ptr, expected, desired);
#endif
}

static void sht30_core_lock_release(volatile uint8_t *ptr)
{
#if defined(_MSC_VER)
    _InterlockedExchange8((volatile char *)ptr, 0);
#else
    __sync_lock_release(ptr);
#endif
}

int sht30_core_try_lock(sht30_dev_t *dev)
{
    if (dev == 0) {
        return -EINVAL;
    }
    if (!sht30_core_lock_cas(&dev->in_use, 0u, 1u)) {
        return -EBUSY;
    }
    return 0;
}

void sht30_core_unlock(sht30_dev_t *dev)
{
    if (dev != 0) {
        sht30_core_lock_release(&dev->in_use);
    }
}

int sht30_core_validate_dev(const sht30_dev_t *dev)
{
    if (dev == 0 || dev->ops == 0 || dev->ops->xfer == 0) {
        return -EINVAL;
    }
    return 0;
}

int sht30_core_map_bus_status(int status)
{
    if (status == 0) {
        return 0;
    }
    if (status < 0) {
        return status;
    }
    return -EIO;
}

void sht30_core_precision_cmd(uint8_t cmd[2], sht30_precision_t precision)
{
    switch (precision) {
    case SHT30_PRECISION_HIGH:
        cmd[0] = 0x24u;
        cmd[1] = 0x00u;
        break;
    case SHT30_PRECISION_MEDIUM:
        cmd[0] = 0x24u;
        cmd[1] = 0x0Bu;
        break;
    case SHT30_PRECISION_LOW:
        cmd[0] = 0x24u;
        cmd[1] = 0x16u;
        break;
    default:
        cmd[0] = 0x24u;
        cmd[1] = 0x00u;
        break;
    }
}

uint32_t sht30_core_measure_delay_ms(const uint8_t cmd[2])
{
    if (cmd[0] == 0x24u && cmd[1] == 0x00u) {
        return 16u;
    }
    if (cmd[0] == 0x24u && cmd[1] == 0x0Bu) {
        return 7u;
    }
    if (cmd[0] == 0x24u && cmd[1] == 0x16u) {
        return 4u;
    }
    if (cmd[0] == 0x30u && cmd[1] == 0xA2u) {
        return 2u;
    }
    return 2u;
}

int sht30_core_xfer_sync(sht30_dev_t *dev, uint8_t *buf, uint16_t len, bool read)
{
    sht30_comm_msg_t msg;
    int rc;

    if (buf == 0 || len == 0u) {
        return -EINVAL;
    }

    rc = sht30_core_validate_dev(dev);
    if (rc != 0) {
        return rc;
    }

    msg.buf = buf;
    msg.len = len;
    msg.flags = (uint8_t)((read ? SHT30_COMM_READ : SHT30_COMM_WRITE) | SHT30_COMM_STOP);

    return sht30_core_map_bus_status(dev->ops->xfer(dev->bus_ctx, &msg, 1u, 0, 0));
}

int sht30_core_validate_response(const uint8_t rx[6])
{
    if (rx == 0) {
        return -EINVAL;
    }
    if (sht30_core_crc8(rx, 2u) != rx[2] || sht30_core_crc8(&rx[3], 2u) != rx[5]) {
        return -EBADMSG;
    }
    return 0;
}

int sht30_core_read_sample_parse(const uint8_t rx[6], sht30_sample_t *out)
{
    uint16_t t_raw;
    uint16_t h_raw;
    int rc;

    if (rx == 0 || out == 0) {
        return -EINVAL;
    }

    rc = sht30_core_validate_response(rx);
    if (rc != 0) {
        return rc;
    }

    t_raw = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    h_raw = (uint16_t)(((uint16_t)rx[3] << 8) | rx[4]);

    out->temperature_c = -45.0f + 175.0f * ((float)t_raw / 65535.0f);
    out->humidity_rh = 100.0f * ((float)h_raw / 65535.0f);

    if (out->humidity_rh < 0.0f) {
        out->humidity_rh = 0.0f;
    }
    if (out->humidity_rh > 100.0f) {
        out->humidity_rh = 100.0f;
    }

    return 0;
}
